using UnityEngine;

public class ParticleDisplay2D : MonoBehaviour
{
	public Mesh mesh;
	public Shader shader;
	public float scale;
	public Gradient colourMap;
	public int gradientResolution;
	public float velocityDisplayMax;

	Material material;
	ComputeBuffer argsBuffer;
	ComputeBuffer dummyStatesBuffer; // only allocated by the Simulation2D path; null for SwarmSimulation.
	Bounds bounds;
	Texture2D gradientTexture;
	bool needsUpdate;

	// Per-state display tint. rgba.a = blend weight: 0 = show velocity gradient, 1 = full override.
	// Calm defaults to alpha 0 so the non-ABM scene and Calm agents are visually unchanged.
	static readonly Vector4[] DefaultStateColors = new Vector4[3]
	{
		new Vector4(1f,   1f,   1f,   0f),    // Calm    - velocity gradient
		new Vector4(1f,   0.2f, 0.2f, 0.85f), // Scared  - red
		new Vector4(0.3f, 0.6f, 1f,   0.85f), // Huddle  - blue
	};


	public void Init(Simulation2D sim)
	{
		material = new Material(shader);
		material.SetBuffer("Positions2D", sim.positionBuffer);
		material.SetBuffer("Velocities", sim.velocityBuffer);
		material.SetBuffer("DensityData", sim.densityBuffer);

		// Shader requires a States buffer; the fluid scene has no states, so bind zeros.
		dummyStatesBuffer = ComputeHelper.CreateStructuredBuffer<uint>(sim.positionBuffer.count);
		material.SetBuffer("States", dummyStatesBuffer);
		material.SetVectorArray("StateColors", DefaultStateColors);

		argsBuffer = ComputeHelper.CreateArgsBuffer(mesh, sim.positionBuffer.count);
		bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
    }
    public void Init(SwarmSimulation sim)
    {
        material = new Material(shader);
        material.SetBuffer("Positions2D", sim.positionBuffer);
        material.SetBuffer("Velocities", sim.velocityBuffer);
        material.SetBuffer("DensityData", sim.densityBuffer);
        material.SetBuffer("States", sim.statesBuffer);
        material.SetVectorArray("StateColors", DefaultStateColors);

        argsBuffer = ComputeHelper.CreateArgsBuffer(mesh, sim.positionBuffer.count);
        bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
    }

    void LateUpdate()
	{
		if (shader != null)
		{
			UpdateSettings();
			Graphics.DrawMeshInstancedIndirect(mesh, 0, material, bounds, argsBuffer);
		}
	}

	void UpdateSettings()
	{
		if (needsUpdate)
		{
			needsUpdate = false;
			TextureFromGradient(ref gradientTexture, gradientResolution, colourMap);
			material.SetTexture("ColourMap", gradientTexture);

			material.SetFloat("scale", scale);
			material.SetFloat("velocityMax", velocityDisplayMax);
		}
	}

	public static void TextureFromGradient(ref Texture2D texture, int width, Gradient gradient, FilterMode filterMode = FilterMode.Bilinear)
	{
		if (texture == null)
		{
			texture = new Texture2D(width, 1);
		}
		else if (texture.width != width)
		{
			texture.Reinitialize(width, 1);
		}
		if (gradient == null)
		{
			gradient = new Gradient();
			gradient.SetKeys(
				new GradientColorKey[] { new GradientColorKey(Color.black, 0), new GradientColorKey(Color.black, 1) },
				new GradientAlphaKey[] { new GradientAlphaKey(1, 0), new GradientAlphaKey(1, 1) }
			);
		}
		texture.wrapMode = TextureWrapMode.Clamp;
		texture.filterMode = filterMode;

		Color[] cols = new Color[width];
		for (int i = 0; i < cols.Length; i++)
		{
			float t = i / (cols.Length - 1f);
			cols[i] = gradient.Evaluate(t);
		}
		texture.SetPixels(cols);
		texture.Apply();
	}

	void OnValidate()
	{
		needsUpdate = true;
	}

	void OnDestroy()
	{
		ComputeHelper.Release(argsBuffer);
		if (dummyStatesBuffer != null) ComputeHelper.Release(dummyStatesBuffer);
	}
}
