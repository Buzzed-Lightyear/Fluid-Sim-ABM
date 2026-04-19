using UnityEngine;
using Unity.Mathematics;

/*
 * Working parameters for accruate fluid sim. particle count: 16K 
 *   iteration per frame: 7
 *   gravity: -9.8
 *   wall bounciness: 0, testing 0 to see if system ever settles. 
 * 
 *   sensor radius: 0.2 
 *   ideal neighbor count: 21 (~18, have air pockets in the top layers, larger values can result in bottom stacking artifact) 
 *   pressure multiplier: 1 (0-1 have largest behavior changes, then less effective w higher value) 
 *   viscosity: 0.03
 *   drag: 0.1
 *   cohesion strength: 0.2 (no effect till larger sensor raidus ~1) 
 * 
 *   collision radius: 0.05 (lower radius seems to have better fluid behavior, gas maybe can use higher values) 
 *   collision stiffness: 100, 50 works too visually 
 * 
 *   -- Speed, force, ineraction parameters have not been tested much. 
 *   max speed: 5, force: 50
 * 
 *   Interaction radius: 2.5
 *   Interaction strength: 75
 * 
 *   Particle Spawner: jitter: 0.02, collision radius: 0.05, count: 16K 
 */

public class SwarmSimulation : MonoBehaviour
{
    public event System.Action SimulationStepCompleted;

    [Header("Simulation Setup")]
    public float timeScale = 1;
    public bool fixedTimeStep;
    public int iterationsPerFrame = 3;
    public float gravity = -9.81f;
    [Range(0, 1)] public float wallBounciness = 0.24f;
    public Vector2 boundsSize = new Vector2(17, 9);
    public Vector2 obstacleSize = new Vector2(0, 0);
    public Vector2 obstacleCentre;

    [Header("Fluid Physics (Flow)")]
    [Tooltip("How far a particle can 'see'. MUST be > 2x Collision Radius.")]
    public float sensorRadius = 0.3f;

    [Tooltip("Ideal number of neighbors. Low (0-2) = Gas/Expanding. High (6-10) = Liquid/Pooling.")]
    public float idealNeighborCount = 8;

    [Tooltip("Soft force to maintain spacing when crowded (Fluid Pressure).")]
    public float pressureMultiplier = 8;

    [Tooltip("How strongly particles match velocity (Syrupiness/Flow).")]
    public float viscosity = 1;

    [Tooltip("Air resistance. 0 = Vacuum, 1 = Molasses.")]
    [Range(0, 1)] public float drag = 0.1f;

    [Tooltip("Pull toward the center of mass of neighbors. 0 = disabled. 0.1-1.0 typical.")]
    public float cohesionStrength = 0.5f;

    [Header("Collision & Stability (Structure)")]
    [Tooltip("Reference value for the sensorRadius sanity check. Actual per-agent collision radii come from ParticleSpawner.defaultCollisionRadius and are stored in a GPU buffer.")]
    public float collisionRadius = 0.05f;

    [Tooltip("How hard solid particles push back when overlapping. Higher = Stiffer/Harder.")]
    public float collisionStiffness = 20; // The "Kick" multiplier

    [Header("Safety Limits")]
    public float maxSpeed = 5;
    [Tooltip("Maximum force allowed. Increase this if particles sink through floor.")]
    public float maxForce = 50;
    // Make sure maxForce is > gravity * some constant, else gravity will make particles to disobey neighbors rules on the border.

    [Header("Interaction")]
    public float interactionRadius = 3;
    public float interactionStrength = 200;

    [Header("Agent States")]
    [Tooltip("Distance from RMB mouse at which Calm agents flip to Scared.")]
    public float panicRadius = 2f;
    [Tooltip("Frames without RMB before Scared agents calm down.")]
    public int calmdownFrames = 120;

    [Header("References")]
    public ComputeShader compute;
    public ParticleSpawner spawner;
    public ParticleDisplay2D display;

    // Buffers & State
    public ComputeBuffer positionBuffer { get; private set; }
    public ComputeBuffer velocityBuffer { get; private set; }
    public ComputeBuffer densityBuffer { get; private set; }
    ComputeBuffer predictedPositionBuffer;
    ComputeBuffer collisionRadiiBuffer;
    public ComputeBuffer statesBuffer { get; private set; }
    ComputeBuffer spatialIndices;
    ComputeBuffer spatialOffsets;
    GPUSort gpuSort;

    // CPU mirror of Agent states - host drives transitions, uploads to GPU each frame.
    uint[] statesCPU;
    int framesSinceRMB;

    // Kernel IDs
    const int externalForcesKernel = 0;
    const int spatialHashKernel = 1;
    const int hardCollisionKernel = 2;
    const int updateBehaviorKernel = 3;
    const int updatePositionKernel = 4;

    bool isPaused;
    bool pauseNextFrame;
    ParticleSpawner.ParticleSpawnData spawnData;
    public int numParticles { get; private set; }

    void Start()
    {
        float deltaTime = 1 / 60f;
        Time.fixedDeltaTime = deltaTime;

        if (spawner == null || display == null)
        {
            Debug.LogError("Assign Spawner and Display references in Inspector!");
            return;
        }

        spawnData = spawner.GetSpawnData();
        numParticles = spawnData.positions.Length;

        // Create Buffers
        positionBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
        predictedPositionBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
        velocityBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
        densityBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
        collisionRadiiBuffer = ComputeHelper.CreateStructuredBuffer<float>(numParticles);
        statesBuffer = ComputeHelper.CreateStructuredBuffer<uint>(numParticles);
        spatialIndices = ComputeHelper.CreateStructuredBuffer<uint3>(numParticles);
        spatialOffsets = ComputeHelper.CreateStructuredBuffer<uint>(numParticles);

        statesCPU = new uint[numParticles];

        SetInitialBufferData(spawnData);
        BindBuffers();

        gpuSort = new GPUSort();
        gpuSort.SetBuffers(spatialIndices, spatialOffsets);
        display.Init(this);
    }

    void Update()
    {
        if (!fixedTimeStep && Time.frameCount > 10)
            RunSimulationFrame(Time.deltaTime);

        if (pauseNextFrame) { isPaused = true; pauseNextFrame = false; }
        HandleInput();
    }

    void FixedUpdate()
    {
        if (fixedTimeStep) RunSimulationFrame(Time.fixedDeltaTime);
    }

    void RunSimulationFrame(float frameTime)
    {
        if (!isPaused)
        {
            float dt = frameTime / iterationsPerFrame * timeScale;
            UpdateStatesCPU();
            UpdateSettings(dt);

            for (int i = 0; i < iterationsPerFrame; i++)
            {
                RunSimulationStep();
                SimulationStepCompleted?.Invoke();
            }
        }
    }

    // Host-side state transitions. Uploads the full states array each frame - the buffer
    // is small enough (uint per particle) that this is cheap. A dedicated GPU kernel is
    // future work; the TASK_C spec explicitly allows CPU-driven transitions for v1.
    void UpdateStatesCPU()
    {
        // Calm=0, Scared=1, Huddle=2 (match AgentState enum in ReferenceSim.cs).
        const uint Calm = 0, Scared = 1, Huddle = 2;

        bool huddleHeld = Input.GetKey(KeyCode.H);
        bool rmbHeld = Input.GetMouseButton(1);

        Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        Vector2[] positionsSnapshot = null;
        if (rmbHeld)
        {
            float2[] posData = new float2[numParticles];
            positionBuffer.GetData(posData);
            positionsSnapshot = new Vector2[numParticles];
            for (int i = 0; i < numParticles; i++)
                positionsSnapshot[i] = new Vector2(posData[i].x, posData[i].y);
        }

        if (rmbHeld) framesSinceRMB = 0;
        else framesSinceRMB++;

        float panicSqr = panicRadius * panicRadius;

        for (int i = 0; i < numParticles; i++)
        {
            if (huddleHeld)
            {
                statesCPU[i] = Huddle;
                continue;
            }

            if (statesCPU[i] == Huddle)
            {
                statesCPU[i] = Calm;
            }

            if (rmbHeld && statesCPU[i] == Calm)
            {
                Vector2 d = positionsSnapshot[i] - mousePos;
                if (d.sqrMagnitude < panicSqr) statesCPU[i] = Scared;
            }
            else if (statesCPU[i] == Scared && framesSinceRMB >= calmdownFrames)
            {
                statesCPU[i] = Calm;
            }
        }

        statesBuffer.SetData(statesCPU);
    }

    void RunSimulationStep()
    {
        ComputeHelper.Dispatch(compute, numParticles, kernelIndex: externalForcesKernel);
        ComputeHelper.Dispatch(compute, numParticles, kernelIndex: spatialHashKernel);
        gpuSort.SortAndCalculateOffsets();
        ComputeHelper.Dispatch(compute, numParticles, kernelIndex: hardCollisionKernel);
        ComputeHelper.Dispatch(compute, numParticles, kernelIndex: updateBehaviorKernel);
        ComputeHelper.Dispatch(compute, numParticles, kernelIndex: updatePositionKernel);
    }

    void BindBuffers()
    {
        ComputeHelper.SetBuffer(compute, positionBuffer, "Positions", externalForcesKernel, updatePositionKernel);
        ComputeHelper.SetBuffer(compute, predictedPositionBuffer, "PredictedPositions", externalForcesKernel, spatialHashKernel, hardCollisionKernel, updateBehaviorKernel);
        ComputeHelper.SetBuffer(compute, velocityBuffer, "Velocities", externalForcesKernel, hardCollisionKernel, updateBehaviorKernel, updatePositionKernel);
        ComputeHelper.SetBuffer(compute, densityBuffer, "Densities", updateBehaviorKernel);
        ComputeHelper.SetBuffer(compute, collisionRadiiBuffer, "CollisionRadii", hardCollisionKernel);
        ComputeHelper.SetBuffer(compute, statesBuffer, "States", updateBehaviorKernel);
        ComputeHelper.SetBuffer(compute, spatialIndices, "SpatialIndices", spatialHashKernel, hardCollisionKernel, updateBehaviorKernel);
        ComputeHelper.SetBuffer(compute, spatialOffsets, "SpatialOffsets", spatialHashKernel, hardCollisionKernel, updateBehaviorKernel);
        compute.SetInt("numParticles", numParticles);
    }

    void UpdateSettings(float deltaTime)
    {
        compute.SetFloat("deltaTime", deltaTime);
        compute.SetFloat("gravity", gravity);
        compute.SetFloat("wallBounciness", wallBounciness);

        compute.SetFloat("sensorRadius", sensorRadius);
        compute.SetFloat("idealNeighborCount", idealNeighborCount);
        compute.SetFloat("pressureMultiplier", pressureMultiplier);
        compute.SetFloat("viscosity", viscosity);
        compute.SetFloat("drag", drag);
        compute.SetFloat("cohesionStrength", cohesionStrength);

        compute.SetFloat("collisionStiffness", collisionStiffness);

        compute.SetFloat("maxSpeed", maxSpeed);
        compute.SetFloat("maxForce", maxForce);

        compute.SetVector("boundsSize", boundsSize);
        compute.SetVector("obstacleSize", obstacleSize);
        compute.SetVector("obstacleCentre", obstacleCentre);

        // Per-state param table: (idealNeighborCount, cohesionMult, speedMult, unused).
        // Calm = baseline; Scared = 0 ideal / no cohesion / 1.5x speed; Huddle = 20 ideal / 3x cohesion / 1x speed.
        Vector4[] stateParams = new Vector4[3];
        stateParams[0] = new Vector4(idealNeighborCount, 1f, 1f, 0f);
        stateParams[1] = new Vector4(0f, 0f, 1.5f, 0f);
        stateParams[2] = new Vector4(20f, 3f, 1f, 0f);
        compute.SetVectorArray("StateParams", stateParams);

        // Interaction
        Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        float interactStr = 0;
        if (Input.GetMouseButton(0)) interactStr = interactionStrength;
        if (Input.GetMouseButton(1)) interactStr = -interactionStrength;

        compute.SetVector("interactionInputPoint", mousePos);
        compute.SetFloat("interactionInputStrength", interactStr);
        compute.SetFloat("interactionInputRadius", interactionRadius);
    }

    void SetInitialBufferData(ParticleSpawner.ParticleSpawnData spawnData)
    {
        float2[] allPoints = new float2[spawnData.positions.Length];
        System.Array.Copy(spawnData.positions, allPoints, spawnData.positions.Length);
        positionBuffer.SetData(allPoints);
        predictedPositionBuffer.SetData(allPoints);
        velocityBuffer.SetData(spawnData.velocities);
        collisionRadiiBuffer.SetData(spawnData.collisionRadii);

        System.Array.Copy(spawnData.states, statesCPU, spawnData.states.Length);
        statesBuffer.SetData(statesCPU);
        framesSinceRMB = calmdownFrames;
    }

    void HandleInput()
    {
        if (Input.GetKeyDown(KeyCode.Space)) isPaused = !isPaused;
        if (Input.GetKeyDown(KeyCode.RightArrow)) { isPaused = false; pauseNextFrame = true; }
        if (Input.GetKeyDown(KeyCode.R))
        {
            isPaused = true;
            SetInitialBufferData(spawnData);
            RunSimulationStep();
            SetInitialBufferData(spawnData);
        }
    }

    // Safety check for the "Blind Driver" bug
    void OnValidate()
    {
        if (sensorRadius < collisionRadius * 2)
        {
            Debug.LogWarning($"Sensor Radius ({sensorRadius}) is too small! It must be at least 2x Collision Radius ({collisionRadius * 2}) or collisions will fail.");
        }
    }

    void OnDestroy()
    {
        ComputeHelper.Release(positionBuffer, predictedPositionBuffer, velocityBuffer, densityBuffer, collisionRadiiBuffer, statesBuffer, spatialIndices, spatialOffsets);
    }

    void OnDrawGizmos()
    {
        Gizmos.color = new Color(0, 1, 0, 0.4f);
        Gizmos.DrawWireCube(Vector2.zero, boundsSize);
        Gizmos.DrawWireCube(obstacleCentre, obstacleSize);
        if (Application.isPlaying)
        {
            Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            if (Input.GetMouseButton(0) || Input.GetMouseButton(1))
            {
                Gizmos.color = Input.GetMouseButton(0) ? Color.green : Color.red;
                Gizmos.DrawWireSphere(mousePos, interactionRadius);
            }
        }
    }
}
