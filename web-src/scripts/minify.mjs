import { readFile, writeFile } from 'node:fs/promises'
import { resolve } from 'node:path'
import { transform } from 'esbuild'

const target = resolve(import.meta.dirname, '../../web/app.js')
const source = await readFile(target, 'utf8')
const result = await transform(source, {
  charset: 'utf8',
  format: 'esm',
  legalComments: 'none',
  minify: true,
  target: 'es2022',
})
await writeFile(target, result.code)
