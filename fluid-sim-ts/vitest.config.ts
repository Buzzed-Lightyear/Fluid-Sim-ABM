import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    include: ['src/ref/__tests__/**/*.test.ts'],
    environment: 'node',
    globals: false,
  }
})
