import { describe, test } from 'vitest'

describe('TunerRunner', () => {
  test('runs tuner if RUN_TUNER=1', async () => {
    if (process.env.RUN_TUNER !== '1') return
    const { FluidTuner } = await import('../FluidTuner.js')
    const tuner = new FluidTuner()
    const result = tuner.run()
    console.log('Tuner result:', JSON.stringify(result, null, 2))
  })
})
