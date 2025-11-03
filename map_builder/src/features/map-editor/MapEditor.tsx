import { useMemo, useRef, useState } from 'react'
import type { ReactNode } from 'react'
import type { CircleArc, MapPoint, SelectedElement, Vector3 } from './types'
import { MapView, type MapViewHandle } from './components/MapView'
import { InspectorPanel } from './components/InspectorPanel'
import { TopBar } from './components/TopBar'
import { buildSegmentsFromPoints, areVectorsClose } from './utils'
import './mapEditor.css'

const SAMPLE_POINTS: MapPoint[] = [
  {
    id: 'pt-1',
    position: { x: 0, y: 0, z: 0 },
  },
  {
    id: 'pt-2',
    position: { x: 10.5, y: 0, z: 0 },
  },
  {
    id: 'pt-3',
    position: { x: 10.5, y: 5.025, z: 0 },
  },
  {
    id: 'pt-4',
    position: { x: 6.8, y: 5.025, z: 0 },
  },
  {
    id: 'pt-5',
    position: { x: 6.8, y: 5.2275, z: 0 },
  },
  {
    id: 'pt-6',
    position: { x: 0, y: 5.2275, z: 0 },
  },
]

const SAMPLE_CIRCLES: CircleArc[] = [
  {
    id: 'cir-1',
    center: { x: 2.81, y: 0.5, z: 0 },
    radius: 0.2,
    startAngle: 0,
    endAngle: 2 * Math.PI,
  },
  {
    id: 'cir-2',
    center: { x: 3.5, y: 1.19, z: 0 },
    radius: 0.2,
    startAngle: 0,
    endAngle: 2 * Math.PI,
  },
  {
    id: 'cir-3',
    center: { x: 4.19, y: 0.5, z: 0 },
    radius: 0.2,
    startAngle: 0,
    endAngle: 2 * Math.PI,
  },
]

type StatusMessage = {
  tone: 'info' | 'error' | 'success'
  text: string
}

function MapEditor() {
  const [points, setPoints] = useState<MapPoint[]>(SAMPLE_POINTS)
  const [circles, setCircles] = useState<CircleArc[]>(SAMPLE_CIRCLES)
  const [selectedElement, setSelectedElement] =
    useState<SelectedElement | null>(null)
  const [status, setStatus] = useState<StatusMessage | null>(null)
  const mapViewRef = useRef<MapViewHandle>(null)

  const selectedPoint = useMemo(() => {
    if (selectedElement?.type !== 'point') return null
    return points.find((point) => point.id === selectedElement.pointId) ?? null
  }, [points, selectedElement])

  const selectedCircle = useMemo(() => {
    if (selectedElement?.type !== 'circle') return null
    return circles.find((circle) => circle.id === selectedElement.circleId) ?? null
  }, [circles, selectedElement])

  const updatePoint = (
    pointId: string,
    updater: (point: MapPoint) => MapPoint,
  ) => {
    setPoints((current) =>
      current.map((point) => (point.id === pointId ? updater(point) : point)),
    )
  }

  const updateCircle = (
    circleId: string,
    updater: (circle: CircleArc) => CircleArc,
  ) => {
    setCircles((items) =>
      items.map((circle) => (circle.id === circleId ? updater(circle) : circle)),
    )
  }

  const reorderPoint = (pointId: string, targetIndex: number) => {
    setPoints((current) => {
      const sourceIndex = current.findIndex((point) => point.id === pointId)
      if (sourceIndex === -1) {
        return current
      }
      const clampedIndex = Math.min(
        Math.max(targetIndex, 0),
        current.length - 1,
      )
      if (clampedIndex === sourceIndex) {
        return current
      }
      const next = [...current]
      const [moved] = next.splice(sourceIndex, 1)
      next.splice(clampedIndex, 0, moved)
      return next
    })
  }

  const addPoint = () => {
    setPoints((current) => {
      const last = current[current.length - 1]
      const base = last?.position ?? { x: 0, y: 0, z: 0 }
      const nextPoint: MapPoint = {
        id: createId(),
        position: {
          x: base.x + 1,
          y: base.y + 1,
          z: base.z,
        },
      }
      setSelectedElement({ type: 'point', pointId: nextPoint.id })
      return [...current, nextPoint]
    })
  }

  const removePoint = (pointId: string) => {
    setPoints((current) => current.filter((point) => point.id !== pointId))
    setSelectedElement((current) => {
      if (current?.type === 'point' && current.pointId === pointId) {
        return null
      }
      return current
    })
  }

  const addCircle = () => {
    setCircles((current) => {
      const last = current[current.length - 1]
      const base = last?.center ?? { x: 0, y: 0, z: 0 }
      const nextCircle: CircleArc = {
        id: createId(),
        center: {
          x: base.x + 1,
          y: base.y + 1,
          z: base.z,
        },
        radius: last?.radius ?? 1,
        startAngle: 0,
        endAngle: 2 * Math.PI,
      }
      setSelectedElement({ type: 'circle', circleId: nextCircle.id })
      return [...current, nextCircle]
    })
  }

  const removeCircle = (circleId: string) => {
    setCircles((current) => current.filter((circle) => circle.id !== circleId))
    setSelectedElement((current) => {
      if (current?.type === 'circle' && current.circleId === circleId) {
        return null
      }
      return current
    })
  }

  const handleLoadFromFiles = async (files: FileList | null) => {
    if (!files || files.length === 0) return

    const fileArray = Array.from(files)
    const newPoints: MapPoint[] = []
    const newCircles: CircleArc[] = []
    let hadLineSegmentFile = false
    let hadCircleFile = false

    const readFile = (file: File) =>
      new Promise<string>((resolve, reject) => {
        const reader = new FileReader()
        reader.onload = () => resolve(String(reader.result))
        reader.onerror = () => reject(reader.error)
        reader.readAsText(file)
      })

    try {
      for (const file of fileArray) {
        const text = await readFile(file)
        if (/line_segments/i.test(file.name)) {
          hadLineSegmentFile = true
          const parsed = parseLineSegmentsCsv(text)
          newPoints.push(...parsed)
        } else if (/circles/i.test(file.name)) {
          hadCircleFile = true
          const parsed = parseCirclesCsv(text)
          newCircles.push(...parsed)
        }
      }

      if (hadLineSegmentFile) {
        setPoints(newPoints)
      }

      if (hadCircleFile) {
        setCircles(newCircles)
      }

      setStatus({
        tone: 'success',
        text: 'マップデータを読み込みました。',
      })
      mapViewRef.current?.fitToContent()
    } catch (error) {
      setStatus({
        tone: 'error',
        text:
          error instanceof Error
            ? `読み込みに失敗しました: ${error.message}`
            : '読み込みに失敗しました。',
      })
    }
  }

  const handleDownload = () => {
    const segments = buildSegmentsFromPoints(points)
    downloadCsv(
      'line_segments.csv',
      ['start_x', 'start_y', 'start_z', 'end_x', 'end_y', 'end_z'],
      segments.map((segment) => [
        segment.start.x,
        segment.start.y,
        segment.start.z,
        segment.end.x,
        segment.end.y,
        segment.end.z,
      ]),
    )

    downloadCsv(
      'circles.csv',
      ['center_x', 'center_y', 'center_z', 'radius', 'start_angle', 'end_angle'],
      circles.map((circle) => [
        circle.center.x,
        circle.center.y,
        circle.center.z,
        circle.radius,
        circle.startAngle,
        circle.endAngle,
      ]),
    )
  }

  return (
    <div className="map-editor">
      <TopBar
        onRequestFileLoad={handleLoadFromFiles}
        onDownload={handleDownload}
      />
      <div className="map-editor__body">
        <InspectorPanel
          circles={circles}
          points={points}
          onAddCircle={addCircle}
          onAddPoint={addPoint}
          onUpdateCircle={updateCircle}
          onUpdatePoint={updatePoint}
          onRemoveCircle={removeCircle}
          onRemovePoint={removePoint}
          onReorderPoint={reorderPoint}
          selectedCircle={selectedCircle}
          selectedElement={selectedElement}
          selectedPoint={selectedPoint}
          status={status}
          onStatusClear={() => setStatus(null)}
          onSelectElement={setSelectedElement}
        />
        <div className="map-editor__viewport">
          <MapView
            ref={mapViewRef}
            circles={circles}
            points={points}
            onSelectElement={setSelectedElement}
            onUpdateCircle={updateCircle}
            onUpdatePoint={updatePoint}
            selectedElement={selectedElement}
          />
          <div className="map-view__hud">
            <HudButton
              label="ズームイン"
              onClick={() => mapViewRef.current?.zoomIn()}
            >
              <ZoomInIcon />
            </HudButton>
            <HudButton
              label="ズームアウト"
              onClick={() => mapViewRef.current?.zoomOut()}
            >
              <ZoomOutIcon />
            </HudButton>
            <HudButton
              label="フィット"
              onClick={() => mapViewRef.current?.fitToContent()}
            >
              <FitViewIcon />
            </HudButton>
          </div>
        </div>
      </div>
    </div>
  )
}

const downloadCsv = (
  filename: string,
  headers: string[],
  rows: (number | string)[][],
) => {
  const csvContent = [headers, ...rows]
    .map((row) => row.join(','))
    .join('\n')
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' })
  const url = URL.createObjectURL(blob)
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = filename
  anchor.click()
  URL.revokeObjectURL(url)
}

const cloneVector = (vector: Vector3): Vector3 => ({
  x: vector.x,
  y: vector.y,
  z: vector.z,
})

const parseLineSegmentsCsv = (text: string): MapPoint[] => {
  const rows = parseCsv(text)
  const [header, ...dataRows] = rows
  const expectedHeader = [
    'start_x',
    'start_y',
    'start_z',
    'end_x',
    'end_y',
    'end_z',
  ]

  if (!header || !matchesHeader(header, expectedHeader)) {
    throw new Error('line_segments.csv のヘッダーが異なります。')
  }

  const segments = dataRows
    .filter((row) => row.length === expectedHeader.length)
    .map((row) => ({
      start: {
        x: Number(row[0]),
        y: Number(row[1]),
        z: Number(row[2]),
      },
      end: {
        x: Number(row[3]),
        y: Number(row[4]),
        z: Number(row[5]),
      },
    }))

  if (segments.length === 0) {
    return []
  }

  const points: MapPoint[] = []

  const pushPoint = (vector: Vector3) => {
    const nextVector = cloneVector(vector)
    if (
      points.length === 0 ||
      !areVectorsClose(points[points.length - 1].position, nextVector)
    ) {
      points.push({
        id: createId(),
        position: nextVector,
      })
    }
  }

  segments.forEach((segment) => {
    pushPoint(segment.start)
    pushPoint(segment.end)
  })

  if (
    points.length > 2 &&
    areVectorsClose(points[points.length - 1].position, points[0].position)
  ) {
    points.pop()
  }

  return points
}

const parseCirclesCsv = (text: string): CircleArc[] => {
  const rows = parseCsv(text)
  const [header, ...dataRows] = rows
  const expectedHeader = [
    'center_x',
    'center_y',
    'center_z',
    'radius',
    'start_angle',
    'end_angle',
  ]

  if (!header || !matchesHeader(header, expectedHeader)) {
    throw new Error('circles.csv のヘッダーが異なります。')
  }

  return dataRows
    .filter((row) => row.length === expectedHeader.length)
    .map((row) => ({
      id: createId(),
      center: {
        x: Number(row[0]),
        y: Number(row[1]),
        z: Number(row[2]),
      },
      radius: Number(row[3]),
      startAngle: Number(row[4]),
      endAngle: Number(row[5]),
    }))
}

const parseCsv = (text: string): string[][] => {
  return text
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter((line) => line.length > 0)
    .map((line) => line.split(',').map((cell) => cell.trim()))
}

const matchesHeader = (header: string[], expected: string[]) => {
  return header.every(
    (cell, index) => cell?.toLowerCase() === expected[index]?.toLowerCase(),
  )
}

const createId = () =>
  typeof crypto !== 'undefined' && 'randomUUID' in crypto
    ? crypto.randomUUID()
    : `id-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`

type HudButtonProps = {
  label: string
  onClick: () => void
  children: ReactNode
}

const HudButton = ({ label, onClick, children }: HudButtonProps) => (
  <button type="button" className="map-view__hud-button" onClick={onClick} aria-label={label}>
    {children}
  </button>
)

const ZoomInIcon = () => (
  <svg viewBox="0 0 20 20" width="16" height="16" aria-hidden="true" focusable="false">
    <circle cx="9" cy="9" r="7" stroke="currentColor" strokeWidth="1.5" fill="none" />
    <line x1="9" y1="5.5" x2="9" y2="12.5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
    <line x1="5.5" y1="9" x2="12.5" y2="9" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
    <line x1="13.5" y1="13.5" x2="18" y2="18" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
  </svg>
)

const ZoomOutIcon = () => (
  <svg viewBox="0 0 20 20" width="16" height="16" aria-hidden="true" focusable="false">
    <circle cx="9" cy="9" r="7" stroke="currentColor" strokeWidth="1.5" fill="none" />
    <line x1="5.5" y1="9" x2="12.5" y2="9" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
    <line x1="13.5" y1="13.5" x2="18" y2="18" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
  </svg>
)

const FitViewIcon = () => (
  <svg viewBox="0 0 20 20" width="16" height="16" aria-hidden="true" focusable="false">
    <rect x="3" y="3" width="14" height="14" rx="2" ry="2" stroke="currentColor" strokeWidth="1.5" fill="none" />
    <polyline
      points="8,3.5 8,6.5 5,6.5"
      stroke="currentColor"
      strokeWidth="1.5"
      fill="none"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
    <polyline
      points="12,16.5 12,13.5 15,13.5"
      stroke="currentColor"
      strokeWidth="1.5"
      fill="none"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
    <polyline
      points="16.5,8 13.5,8 13.5,5"
      stroke="currentColor"
      strokeWidth="1.5"
      fill="none"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
    <polyline
      points="3.5,12 6.5,12 6.5,15"
      stroke="currentColor"
      strokeWidth="1.5"
      fill="none"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
  </svg>
)

export default MapEditor
