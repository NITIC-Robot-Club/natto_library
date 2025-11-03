export type Vector3 = {
  x: number
  y: number
  z: number
}

export type SelectedElement =
  | { type: 'line'; lineId: string; endpoint?: 'start' | 'end' }
  | { type: 'circle'; circleId: string }

export type StatusMessage = {
  tone: 'info' | 'error' | 'success'
  text: string
}

export type ViewportState = {
  offsetX: number
  offsetY: number
  scale: number
}

export type LineSegment = {
  id: string
  start: Vector3
  end: Vector3
}

export type CircleArc = {
  id: string
  center: Vector3
  radius: number
  startAngle: number
  endAngle: number
}
