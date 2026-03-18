import { defineConfig } from 'vite'

export default defineConfig({
  server: { port: 8080 },
  // Relative base so the built index.html works when served from any path
  base: './',
})
