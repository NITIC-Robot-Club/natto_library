import {
  forwardRef,
  useCallback,
  useEffect,
  useImperativeHandle,
  useRef,
  useState,
} from 'react'
import type {
  CircleArc,
  LineSegment,
  SelectedElement,
  Vector3,
  ViewportState,
} from '../types'

type MapViewProps = {
  lines: LineSegment[]
  circles: CircleArc[]
  selectedElement: SelectedElement | null
  onSelectElement: (element: SelectedElement | null) => void
  onBeginInteraction?: () => void
  onUpdateLine: (
    lineId: string,
    updater: (line: LineSegment) => LineSegment,
  ) => void
  onUpdateCircle: (
    circleId: string,
    updater: (circle: CircleArc) => CircleArc,
  ) => void
}

export type MapViewHandle = {
  fitToContent: () => void
  zoomIn: () => void
  zoomOut: () => void
  resetZoom: () => void
  rotateClockwise: () => void
  resetRotation: () => void
}

type DragState =
  | {
      kind: 'line-endpoint'
      lineId: string
      endpoint: 'start' | 'end'
      pointerId: number
    }
  | {
      kind: 'line-translate'
      lineId: string
      pointerId: number
      originWorldX: number
      originWorldY: number
      initialStart: Vector3
      initialEnd: Vector3
    }
  | {
      kind: 'circle-center'
      circleId: string
      pointerId: number
    }
  | {
      kind: 'circle-radius'
      circleId: string
      pointerId: number
    }
  | {
      kind: 'pan'
      pointerId: number
      originX: number
      originY: number
      startOffsetX: number
      startOffsetY: number
      hasMoved: boolean
    }

const MIN_SCALE = 10
const MAX_SCALE = 350
const LINE_HIT_STROKE_WIDTH = 16
export const MapView = forwardRef<MapViewHandle, MapViewProps>(
  (
    {
      lines,
      circles,
      selectedElement,
      onSelectElement,
      onBeginInteraction,
      onUpdateLine,
      onUpdateCircle,
    },
    ref,
  ) => {
    const containerRef = useRef<HTMLDivElement | null>(null)
    const svgRef = useRef<SVGSVGElement | null>(null)
    const [viewport, setViewport] = useState<ViewportState>({
      offsetX: 120,
      offsetY: 200,
      scale: 60,
      rotation: 0,
    })
    const [containerSize, setContainerSize] = useState({ width: 1, height: 1 })
    const [dragState, setDragState] = useState<DragState | null>(null)
    const hasManualViewport = useRef(false)
    const initialFitDone = useRef(false)
    const linesRef = useLatest(lines)
    const circlesRef = useLatest(circles)

    useEffect(() => {
      if (!containerRef.current) return
      const observer = new ResizeObserver((entries) => {
        const entry = entries[0]
        if (entry?.contentRect) {
          setContainerSize({
            width: entry.contentRect.width,
            height: entry.contentRect.height,
          })
        }
      })
      observer.observe(containerRef.current)
      return () => observer.disconnect()
    }, [])

    const fitToContent = useCallback(() => {
      const bounds = computeBounds(linesRef.current, circlesRef.current)
      const padding = 120
      const width = containerSize.width
      const height = containerSize.height
      const worldWidth = Math.max(bounds.maxX - bounds.minX, 1)
      const worldHeight = Math.max(bounds.maxY - bounds.minY, 1)
      const scaleX = (width - padding * 2) / worldWidth
      const scaleY = (height - padding * 2) / worldHeight
      const scale = clamp(Math.min(scaleX, scaleY), MIN_SCALE, MAX_SCALE)

      setViewport({
        scale,
        offsetX: padding - bounds.minX * scale,
        offsetY: padding - bounds.minY * scale,
        rotation: 0,
      })
      hasManualViewport.current = false
    }, [circlesRef, containerSize.height, containerSize.width, linesRef])

    useEffect(() => {
      if (initialFitDone.current) return
      if (containerSize.width > 1 && containerSize.height > 1) {
        fitToContent()
        initialFitDone.current = true
      }
    }, [containerSize.height, containerSize.width, fitToContent])

    const zoom = useCallback(
      (factor: number) => {
        hasManualViewport.current = true
        setViewport((current) => {
          const newScale = clamp(current.scale * factor, MIN_SCALE, MAX_SCALE)
          const centerX = containerSize.width / 2
          const centerY = containerSize.height / 2
          const centerWorld = screenToWorld(
            centerX,
            centerY,
            current,
          )
          const newOffsetX = centerX - projectX(centerWorld.x) * newScale
          const newOffsetY = centerY - centerWorld.y * newScale
          return {
            scale: newScale,
            offsetX: newOffsetX,
            offsetY: newOffsetY,
            rotation: current.rotation,
          }
        })
      },
      [containerSize.height, containerSize.width],
    )

    const rotateViewport = useCallback(
      (delta: number) => {
        setViewport((current) => {
          const nextRotation = normalizeAngle(current.rotation + delta)
          if (containerSize.width <= 0 || containerSize.height <= 0) {
            return {
              ...current,
              rotation: nextRotation,
            }
          }
          const centerX = containerSize.width / 2
          const centerY = containerSize.height / 2
          const centerWorld = screenToWorld(centerX, centerY, current)
          const zeroOffsetViewport: ViewportState = {
            ...current,
            rotation: nextRotation,
            offsetX: 0,
            offsetY: 0,
          }
          const projectedCenter = worldToScreen(centerWorld, zeroOffsetViewport)
          return {
            ...current,
            rotation: nextRotation,
            offsetX: centerX - projectedCenter.x,
            offsetY: centerY - projectedCenter.y,
          }
        })
        hasManualViewport.current = true
      },
      [containerSize.height, containerSize.width],
    )

    useImperativeHandle(
      ref,
      () => ({
        fitToContent,
        zoomIn: () => zoom(1.2),
        zoomOut: () => zoom(1 / 1.2),
        resetZoom: () => {
          setViewport({
            scale: 60,
            offsetX: 120,
            offsetY: 200,
            rotation: viewport.rotation,
          })
          hasManualViewport.current = false
        },
        rotateClockwise: () => {
          rotateViewport(Math.PI / 2)
        },
        resetRotation: () => {
          rotateViewport(-viewport.rotation)
        },
      }),
      [zoom, fitToContent, rotateViewport, viewport.rotation],
    )

    useEffect(() => {
      if (dragState?.kind !== 'pan') return
      hasManualViewport.current = true
    }, [dragState])

    const handlePointerMove = (event: React.PointerEvent<SVGSVGElement>) => {
      if (!dragState || !svgRef.current) return

      event.preventDefault()
      const rect = svgRef.current.getBoundingClientRect()
      const screenX = event.clientX - rect.left
      const screenY = event.clientY - rect.top

      if (dragState.kind === 'line-endpoint') {
        const line =
          linesRef.current.find((item) => item.id === dragState.lineId) ?? null
        if (!line) return
        const worldPoint = screenToWorld(screenX, screenY, viewport)

        onUpdateLine(dragState.lineId, (prev) => ({
          ...prev,
          [dragState.endpoint]: {
            x: worldPoint.x,
            y: worldPoint.y,
            z: prev[dragState.endpoint].z,
          },
        }))
      } else if (dragState.kind === 'line-translate') {
        const worldPoint = screenToWorld(screenX, screenY, viewport)
        const deltaX = worldPoint.x - dragState.originWorldX
        const deltaY = worldPoint.y - dragState.originWorldY

        onUpdateLine(dragState.lineId, (prev) => ({
          ...prev,
          start: {
            ...prev.start,
            x: dragState.initialStart.x + deltaX,
            y: dragState.initialStart.y + deltaY,
          },
          end: {
            ...prev.end,
            x: dragState.initialEnd.x + deltaX,
            y: dragState.initialEnd.y + deltaY,
          },
        }))
      } else if (dragState.kind === 'circle-center') {
        const circle =
          circlesRef.current.find((item) => item.id === dragState.circleId) ??
          null
        if (!circle) return
        const worldPoint = screenToWorld(screenX, screenY, viewport)
        onUpdateCircle(dragState.circleId, (prev) => ({
          ...prev,
          center: {
            ...prev.center,
            x: worldPoint.x,
            y: worldPoint.y,
          },
        }))
      } else if (dragState.kind === 'circle-radius') {
        const circle =
          circlesRef.current.find((item) => item.id === dragState.circleId) ??
          null
        if (!circle) return
        const worldPoint = screenToWorld(screenX, screenY, viewport)
        const dx = worldPoint.x - circle.center.x
        const dy = worldPoint.y - circle.center.y
        const radius = Math.max(Math.hypot(dx, dy), 0.001)
        onUpdateCircle(dragState.circleId, (prev) => ({
          ...prev,
          radius,
        }))
      } else if (dragState.kind === 'pan') {
        const deltaX = event.clientX - dragState.originX
        const deltaY = event.clientY - dragState.originY
        if (deltaX !== 0 || deltaY !== 0) {
          setDragState((currentState) =>
            currentState && currentState.kind === 'pan'
              ? { ...currentState, hasMoved: true }
              : currentState,
          )
        }
        setViewport((current) => ({
          ...current,
          offsetX: dragState.startOffsetX + deltaX,
          offsetY: dragState.startOffsetY + deltaY,
        }))
      }
    }

    const endDrag = (options?: { cancelled?: boolean }) => {
      setDragState((currentState) => {
        if (
          currentState?.kind === 'pan' &&
          !currentState.hasMoved &&
          !options?.cancelled
        ) {
          onSelectElement(null)
        }
        return null
      })
    }

    const handlePointerUp = () => {
      endDrag()
    }

    const handleBackgroundPointerDown = (
      event: React.PointerEvent<SVGRectElement>,
    ) => {
      if (event.button !== 0) {
        return
      }

      event.preventDefault()
      hasManualViewport.current = true
      const pointerId = event.pointerId
      event.currentTarget.setPointerCapture(pointerId)
      setDragState({
        kind: 'pan',
        pointerId,
        originX: event.clientX,
        originY: event.clientY,
        startOffsetX: viewport.offsetX,
        startOffsetY: viewport.offsetY,
        hasMoved: false,
      })
    }

    useEffect(() => {
      const svgNode = svgRef.current
      if (!svgNode) return

      const handlePointerLeave = () => setDragState(null)
      svgNode.addEventListener('pointerleave', handlePointerLeave)
      return () => svgNode.removeEventListener('pointerleave', handlePointerLeave)
    }, [])

    const handleWheel = (event: React.WheelEvent<SVGSVGElement>) => {
      if (!event.ctrlKey) {
        event.preventDefault()
        const factor = event.deltaY < 0 ? 1.1 : 1 / 1.1
        hasManualViewport.current = true
        setViewport((current) => {
          const newScale = clamp(current.scale * factor, MIN_SCALE, MAX_SCALE)
          const rect = svgRef.current?.getBoundingClientRect()
          const screenX = rect ? event.clientX - rect.left : containerSize.width / 2
          const screenY = rect ? event.clientY - rect.top : containerSize.height / 2
          const worldPoint = screenToWorld(screenX, screenY, current)
          return {
            ...current,
            scale: newScale,
            offsetX: screenX - projectX(worldPoint.x) * newScale,
            offsetY: screenY - worldPoint.y * newScale,
          }
        })
      }
    }

    const renderSegments = lines.map((line) => {
      const start = worldToScreen(line.start, viewport)
      const end = worldToScreen(line.end, viewport)
      const isSelected =
        selectedElement?.type === 'line' && selectedElement.lineId === line.id
      const strokeWidth = isSelected ? 3.2 : 2.2

      const handleLinePointerDown = (event: React.PointerEvent<SVGLineElement>) => {
        if (event.button !== 0) return
        event.stopPropagation()
        const pointerId = event.pointerId
        event.currentTarget.setPointerCapture(pointerId)
        onSelectElement({ type: 'line', lineId: line.id })
        onBeginInteraction?.()

        const rect = svgRef.current?.getBoundingClientRect()
        if (!rect) return
        const screenX = event.clientX - rect.left
        const screenY = event.clientY - rect.top
        const worldPoint = screenToWorld(screenX, screenY, viewport)

        setDragState({
          kind: 'line-translate',
          lineId: line.id,
          pointerId,
          originWorldX: worldPoint.x,
          originWorldY: worldPoint.y,
          initialStart: { ...line.start },
          initialEnd: { ...line.end },
        })
      }

      return (
        <g key={line.id} className="map-view__line">
          <line
            x1={start.x}
            y1={start.y}
            x2={end.x}
            y2={end.y}
            stroke={isSelected ? '#58d0ff' : '#99a1b3'}
            strokeWidth={strokeWidth}
            strokeLinecap="round"
            pointerEvents="none"
          />
          <line
            x1={start.x}
            y1={start.y}
            x2={end.x}
            y2={end.y}
            stroke="rgba(0, 0, 0, 0)"
            strokeWidth={LINE_HIT_STROKE_WIDTH}
            strokeLinecap="round"
            pointerEvents="stroke"
            onPointerDown={handleLinePointerDown}
          />
        </g>
      )
    })

    const orderedLineHandles =
      selectedElement?.type === 'line'
        ? [
            ...lines.filter((line) => line.id !== selectedElement.lineId),
            lines.find((line) => line.id === selectedElement.lineId) ?? null,
          ].filter((line): line is LineSegment => line !== null)
        : lines

    const renderLineHandles = orderedLineHandles.flatMap((line) => {
      const isSelectedLine =
        selectedElement?.type === 'line' && selectedElement.lineId === line.id
      const activeEndpoint = isSelectedLine ? selectedElement.endpoint : undefined

      return (['start', 'end'] as const).map((endpoint) => {
        const screenPoint = worldToScreen(line[endpoint], viewport)
        const isActive = isSelectedLine && activeEndpoint === endpoint

        return (
          <g key={`${line.id}-${endpoint}`} className="map-view__point">
            <circle
              className={`map-view__handle map-view__handle--point ${
                isActive ? 'is-active' : ''
              }`}
              cx={screenPoint.x}
              cy={screenPoint.y}
              r={8}
              stroke="rgba(255,255,255,0.85)"
              strokeWidth={2}
              fill="rgba(0,0,0,0.45)"
              onPointerDown={(event) => {
                event.stopPropagation()
                const pointerId = event.pointerId
                event.currentTarget.setPointerCapture(pointerId)
                onBeginInteraction?.()
                setDragState({
                  kind: 'line-endpoint',
                  lineId: line.id,
                  endpoint,
                  pointerId,
                })
                onSelectElement({ type: 'line', lineId: line.id, endpoint })
              }}
            />
          </g>
        )
      })
    })

    const renderCircleArc = (circle: CircleArc) => {
      const isFullCircle =
        Math.abs(circle.endAngle - circle.startAngle) >= Math.PI * 2 - 0.0001
      const isSelected =
        selectedElement?.type === 'circle' &&
        selectedElement.circleId === circle.id
      const center = worldToScreen(circle.center, viewport)
      const radiusPixels = circle.radius * viewport.scale
      const arcElements = []

      if (isFullCircle) {
        arcElements.push(
          <circle
            key={`${circle.id}-full`}
            cx={center.x}
            cy={center.y}
            r={radiusPixels}
            stroke={isSelected ? '#ffd166' : '#e2a95f'}
            strokeWidth={isSelected ? 3 : 2.2}
            fill="none"
            onPointerDown={(event) => {
              event.stopPropagation()
              onSelectElement({ type: 'circle', circleId: circle.id })
            }}
          />,
        )
      } else {
        const path = buildArcPath(circle, viewport)
        if (path) {
          arcElements.push(
            <path
              key={`${circle.id}-arc`}
              d={path}
              stroke={isSelected ? '#ffd166' : '#e2a95f'}
              strokeWidth={isSelected ? 3 : 2.2}
              fill="none"
              onPointerDown={(event) => {
                event.stopPropagation()
                onSelectElement({ type: 'circle', circleId: circle.id })
              }}
            />,
          )
        }
      }

      const radiusHandlePoint = {
        x: circle.center.x + circle.radius * Math.cos(circle.startAngle),
        y: circle.center.y + circle.radius * Math.sin(circle.startAngle),
        z: circle.center.z,
      }
      const radiusHandle = worldToScreen(radiusHandlePoint, viewport)

      return (
        <g key={circle.id} className="map-view__circle">
          {arcElements}
          {renderHandle({
            key: `${circle.id}-center-handle`,
            cx: center.x,
            cy: center.y,
            className: `map-view__handle--center ${isSelected ? 'is-active' : ''}`,
            onPointerDown: (event) => {
              event.stopPropagation()
              const pointerId = event.pointerId
              event.currentTarget.setPointerCapture(pointerId)
              onBeginInteraction?.()
              setDragState({
                kind: 'circle-center',
                circleId: circle.id,
                pointerId,
              })
              onSelectElement({ type: 'circle', circleId: circle.id })
            },
          })}
          {renderHandle({
            key: `${circle.id}-radius-handle`,
            cx: radiusHandle.x,
            cy: radiusHandle.y,
            className: `map-view__handle--radius ${isSelected ? 'is-active' : ''}`,
            onPointerDown: (event) => {
              event.stopPropagation()
              const pointerId = event.pointerId
              event.currentTarget.setPointerCapture(pointerId)
              onBeginInteraction?.()
              setDragState({
                kind: 'circle-radius',
                circleId: circle.id,
                pointerId,
              })
              onSelectElement({ type: 'circle', circleId: circle.id })
            },
          })}
        </g>
      )
    }

    return (
      <div className="map-view" ref={containerRef}>
        <svg
          ref={svgRef}
          className="map-view__svg"
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
          onPointerCancel={() => endDrag({ cancelled: true })}
          onWheel={handleWheel}
          onContextMenu={(event) => event.preventDefault()}
        >
          <defs>
            <pattern
              id="mapGrid"
              width={viewport.scale}
              height={viewport.scale}
              patternUnits="userSpaceOnUse"
              patternTransform={`rotate(${(viewport.rotation * 180) / Math.PI})`}
            >
              <path
                d={`M ${viewport.scale} 0 L 0 0 0 ${viewport.scale}`}
                fill="none"
                stroke="rgba(255,255,255,0.08)"
                strokeWidth="1"
              />
            </pattern>
          </defs>
          <rect
            className="map-view__background"
            x={0}
            y={0}
            width="100%"
            height="100%"
            onPointerDown={handleBackgroundPointerDown}
            fill="url(#mapGrid)"
          />
          <g className="map-view__content">
            {renderSegments}
            {circles.map((circle) => renderCircleArc(circle))}
            {renderLineHandles}
          </g>
        </svg>
      </div>
    )
  },
)

MapView.displayName = 'MapView'

const clamp = (value: number, min: number, max: number) =>
  Math.min(Math.max(value, min), max)

const projectX = (x: number) => -x
const unprojectX = (value: number) => -value

const rotatePoint = (x: number, y: number, rotation: number) => {
  const cos = Math.cos(rotation)
  const sin = Math.sin(rotation)
  return {
    x: x * cos - y * sin,
    y: x * sin + y * cos,
  }
}

const inverseRotatePoint = (x: number, y: number, rotation: number) => {
  const cos = Math.cos(rotation)
  const sin = Math.sin(rotation)
  return {
    x: x * cos + y * sin,
    y: -x * sin + y * cos,
  }
}

const worldToScreen = (point: { x: number; y: number }, viewport: ViewportState) => {
  const projectedX = projectX(point.x)
  const rotated = rotatePoint(projectedX, point.y, viewport.rotation)
  return {
    x: rotated.x * viewport.scale + viewport.offsetX,
    y: rotated.y * viewport.scale + viewport.offsetY,
  }
}

const screenToWorld = (x: number, y: number, viewport: ViewportState) => {
  const scaledX = (x - viewport.offsetX) / viewport.scale
  const scaledY = (y - viewport.offsetY) / viewport.scale
  const unrotated = inverseRotatePoint(scaledX, scaledY, viewport.rotation)
  return {
    x: unprojectX(unrotated.x),
    y: unrotated.y,
  }
}

const normalizeAngle = (angle: number) => {
  const twoPi = Math.PI * 2
  const normalized = angle % twoPi
  return normalized < 0 ? normalized + twoPi : normalized
}

const computeBounds = (lines: LineSegment[], circles: CircleArc[]) => {
  let minX = Number.POSITIVE_INFINITY
  let minY = Number.POSITIVE_INFINITY
  let maxX = Number.NEGATIVE_INFINITY
  let maxY = Number.NEGATIVE_INFINITY

  lines.forEach((line) => {
    const endpoints = [line.start, line.end]
    endpoints.forEach((endpoint) => {
      const projectedX = projectX(endpoint.x)
      minX = Math.min(minX, projectedX)
      maxX = Math.max(maxX, projectedX)
      minY = Math.min(minY, endpoint.y)
      maxY = Math.max(maxY, endpoint.y)
    })
  })

  circles.forEach((circle) => {
    const projectedCenterX = projectX(circle.center.x)
    minX = Math.min(minX, projectedCenterX - circle.radius)
    maxX = Math.max(maxX, projectedCenterX + circle.radius)
    minY = Math.min(minY, circle.center.y - circle.radius)
    maxY = Math.max(maxY, circle.center.y + circle.radius)
  })

  if (lines.length === 0 && circles.length === 0) {
    minX = -5
    maxX = 5
    minY = -5
    maxY = 5
  }

  return { minX, maxX, minY, maxY }
}

const renderHandle = ({
  key,
  cx,
  cy,
  onPointerDown,
  className,
}: {
  key: string
  cx: number
  cy: number
  onPointerDown: (event: React.PointerEvent<SVGCircleElement>) => void
  className?: string
}) => (
  <circle
    key={key}
    className={`map-view__handle ${className ?? ''}`}
    cx={cx}
    cy={cy}
    r={8}
    stroke="rgba(255,255,255,0.85)"
    strokeWidth={2}
    fill="rgba(0,0,0,0.45)"
    onPointerDown={onPointerDown}
  />
)

const buildArcPath = (circle: CircleArc, viewport: ViewportState) => {
  const span = circle.endAngle - circle.startAngle
  if (Math.abs(span) < 0.0001) return ''

  const start = polarToCartesian(circle, circle.startAngle, viewport)
  const end = polarToCartesian(circle, circle.endAngle, viewport)
  const radius = circle.radius * viewport.scale
  const largeArcFlag = Math.abs(span) > Math.PI ? 1 : 0
  const sweepFlag = span >= 0 ? 1 : 0

  return `M ${start.x} ${start.y} A ${radius} ${radius} 0 ${largeArcFlag} ${sweepFlag} ${end.x} ${end.y}`
}

const polarToCartesian = (
  circle: CircleArc,
  angle: number,
  viewport: ViewportState,
) => {
  const x = circle.center.x + circle.radius * Math.cos(angle)
  const y = circle.center.y + circle.radius * Math.sin(angle)
  return worldToScreen({ x, y }, viewport)
}

const useLatest = <T,>(value: T) => {
  const ref = useRef(value)
  useEffect(() => {
    ref.current = value
  }, [value])
  return ref
}
