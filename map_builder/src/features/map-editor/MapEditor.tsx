import { useMemo, useRef, useState } from 'react'
import type { ReactNode } from 'react'
import type {
  CircleArc,
  LineSegment,
  SelectedElement,
  StatusMessage,
} from './types'
import { MapView, type MapViewHandle } from './components/MapView'
import { GeometryListPanel } from './components/GeometryListPanel'
import { InspectorPanel } from './components/InspectorPanel'
import { TopBar } from './components/TopBar'
import './mapEditor.css'

const SAMPLE_LINES: LineSegment[] = [
  {
    id: 'seg-1',
    start: { x: 0, y: 0, z: 0 },
    end: { x: 10.5, y: 0, z: 0 },
  },
  {
    id: 'seg-2',
    start: { x: 10.5, y: 0, z: 0 },
    end: { x: 10.5, y: 5.025, z: 0 },
  },
  {
    id: 'seg-3',
    start: { x: 10.5, y: 5.025, z: 0 },
    end: { x: 6.8, y: 5.025, z: 0 },
  },
  {
    id: 'seg-4',
    start: { x: 6.8, y: 5.025, z: 0 },
    end: { x: 6.8, y: 5.2275, z: 0 },
  },
  {
    id: 'seg-5',
    start: { x: 6.8, y: 5.2275, z: 0 },
    end: { x: 0, y: 5.2275, z: 0 },
  },
  {
    id: 'seg-6',
    start: { x: 0, y: 5.2275, z: 0 },
    end: { x: 0, y: 0, z: 0 },
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

function MapEditor() {
  const [lines, setLines] = useState<LineSegment[]>(SAMPLE_LINES)
  const [circles, setCircles] = useState<CircleArc[]>(SAMPLE_CIRCLES)
  const [selectedElement, setSelectedElement] =
    useState<SelectedElement | null>(null)
  const [status, setStatus] = useState<StatusMessage | null>(null)
  const mapViewRef = useRef<MapViewHandle>(null)

  const selectedLine = useMemo(() => {
    if (selectedElement?.type !== 'line') return null
    return lines.find((line) => line.id === selectedElement.lineId) ?? null
  }, [lines, selectedElement])

  const selectedCircle = useMemo(() => {
    if (selectedElement?.type !== 'circle') return null
    return circles.find((circle) => circle.id === selectedElement.circleId) ?? null
  }, [circles, selectedElement])

  const updateLine = (
    lineId: string,
    updater: (line: LineSegment) => LineSegment,
  ) => {
    setLines((current) =>
      current.map((line) => (line.id === lineId ? updater(line) : line)),
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

  const addLine = () => {
    setLines((current) => {
      const last = current[current.length - 1]
      const baseStart = last ? last.end : { x: 0, y: 0, z: 0 }
      const nextLine: LineSegment = {
        id: createId(),
        start: { ...baseStart },
        end: {
          x: baseStart.x + 1,
          y: baseStart.y,
          z: baseStart.z,
        },
      }
      setSelectedElement({ type: 'line', lineId: nextLine.id })
      return [...current, nextLine]
    })
  }

  const removeLine = (lineId: string) => {
    setLines((current) => current.filter((line) => line.id !== lineId))
    setSelectedElement((current) => {
      if (current?.type === 'line' && current.lineId === lineId) {
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
    const newLines: LineSegment[] = []
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
          newLines.push(...parsed)
        } else if (/circles/i.test(file.name)) {
          hadCircleFile = true
          const parsed = parseCirclesCsv(text)
          newCircles.push(...parsed)
        }
      }

      if (hadLineSegmentFile) {
        setLines(newLines)
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
    downloadCsv(
      'line_segments.csv',
      ['start_x', 'start_y', 'start_z', 'end_x', 'end_y', 'end_z'],
      lines.map((segment) => [
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
        <GeometryListPanel
          circles={circles}
          lines={lines}
          onAddCircle={addCircle}
          onAddLine={addLine}
          onRemoveCircle={removeCircle}
          onRemoveLine={removeLine}
          onSelectElement={setSelectedElement}
          selectedElement={selectedElement}
          status={status}
          onStatusClear={() => setStatus(null)}
        />
        <div className="map-editor__viewport">
          <MapView
            ref={mapViewRef}
            circles={circles}
            lines={lines}
            onSelectElement={setSelectedElement}
            onUpdateCircle={updateCircle}
            onUpdateLine={updateLine}
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
        <InspectorPanel
          selectedCircle={selectedCircle}
          selectedLine={selectedLine}
          onUpdateCircle={updateCircle}
          onUpdateLine={updateLine}
        />
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

const parseLineSegmentsCsv = (text: string): LineSegment[] => {
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

  return dataRows
    .filter((row) => row.length === expectedHeader.length)
    .map((row) => ({
      id: createId(),
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
