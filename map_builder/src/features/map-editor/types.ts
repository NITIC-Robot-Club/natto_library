export type Vector3 = {
  x: number
  y: number
  z: number
}

export type MapPoint = {
  id: string
  position: Vector3
}

export type SelectedElement =
  | { type: 'point'; pointId: string }
  | { type: 'circle'; circleId: string }

export type ViewportState = {
  offsetX: number
  offsetY: number
  scale: number
}

export type CircleArc = {
  id: string
  center: Vector3
  radius: number
  startAngle: number
  endAngle: number
}
