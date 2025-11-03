import type { MapPoint, Vector3 } from './types'

export type DerivedLineSegment = {
  id: string
  start: Vector3
  end: Vector3
  startPointId: string
  endPointId: string
}

export const buildSegmentsFromPoints = (
  points: MapPoint[],
): DerivedLineSegment[] => {
  if (points.length < 2) {
    return []
  }

  const segments: DerivedLineSegment[] = []

  for (let index = 0; index < points.length - 1; index += 1) {
    const current = points[index]
    const next = points[index + 1]
    segments.push({
      id: `seg-${current.id}-${next.id}`,
      start: current.position,
      end: next.position,
      startPointId: current.id,
      endPointId: next.id,
    })
  }

  if (points.length >= 3) {
    const last = points[points.length - 1]
    const first = points[0]
    segments.push({
      id: `seg-${last.id}-${first.id}`,
      start: last.position,
      end: first.position,
      startPointId: last.id,
      endPointId: first.id,
    })
  }

  return segments
}

export const areVectorsClose = (a: Vector3, b: Vector3, tolerance = 1e-6) => {
  return (
    Math.abs(a.x - b.x) <= tolerance &&
    Math.abs(a.y - b.y) <= tolerance &&
    Math.abs(a.z - b.z) <= tolerance
  )
}
