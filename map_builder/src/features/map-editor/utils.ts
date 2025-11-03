import type { Vector3 } from './types'

export const areVectorsClose = (a: Vector3, b: Vector3, tolerance = 1e-6) => {
  return (
    Math.abs(a.x - b.x) <= tolerance &&
    Math.abs(a.y - b.y) <= tolerance &&
    Math.abs(a.z - b.z) <= tolerance
  )
}
