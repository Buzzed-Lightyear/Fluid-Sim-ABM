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
 *   viscosity: 0.3
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
 *   Particle Spawner: jitter: 0.02, collision radius: 0.05
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
    ComputeBuffer spatialIndices;
    ComputeBuffer spatialOffsets;
    GPUSort gpuSort;

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
        spatialIndices = ComputeHelper.CreateStructuredBuffer<uint3>(numParticles);
        spatialOffsets = ComputeHelper.CreateStructuredBuffer<uint>(numParticles);

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
            UpdateSettings(dt);

            for (int i = 0; i < iterationsPerFrame; i++)
            {
                RunSimulationStep();
                SimulationStepCompleted?.Invoke();
            }
        }
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
        ComputeHelper.Release(positionBuffer, predictedPositionBuffer, velocityBuffer, densityBuffer, collisionRadiiBuffer, spatialIndices, spatialOffsets);
    }

    void OnDrawGizmos()
    {
        Gizmos.color = new Color(0, 1, 0, 0.4f);
        Gizmos.DrawWireCube(Vector2.zero, boundsSize);
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
