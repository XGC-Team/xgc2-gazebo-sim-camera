import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { resolve } from 'node:path'

export default defineConfig({
  define: {
    'process.env.NODE_ENV': JSON.stringify('production'),
  },
  plugins: [react()],
  build: {
    emptyOutDir: false,
    lib: {
      entry: resolve(import.meta.dirname, 'src/main.tsx'),
      formats: ['es'],
      fileName: () => 'app.js',
      cssFileName: 'style',
    },
    minify: false,
    outDir: resolve(import.meta.dirname, '../web'),
    rollupOptions: {
      output: {
        assetFileNames: 'style.[ext]',
        inlineDynamicImports: true,
      },
    },
  },
})
