import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
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

type EditorState = {
  lines: LineSegment[]
  circles: CircleArc[]
}

const cloneLineSegment = (line: LineSegment): LineSegment => ({
  ...line,
  start: { ...line.start },
  end: { ...line.end },
})

const cloneCircleArc = (circle: CircleArc): CircleArc => ({
  ...circle,
  center: { ...circle.center },
})

const cloneEditorState = (state: EditorState): EditorState => ({
  lines: state.lines.map(cloneLineSegment),
  circles: state.circles.map(cloneCircleArc),
})

const isLineEqual = (a: LineSegment, b: LineSegment) =>
  a.start.x === b.start.x &&
  a.start.y === b.start.y &&
  a.start.z === b.start.z &&
  a.end.x === b.end.x &&
  a.end.y === b.end.y &&
  a.end.z === b.end.z

const isCircleEqual = (a: CircleArc, b: CircleArc) =>
  a.center.x === b.center.x &&
  a.center.y === b.center.y &&
  a.center.z === b.center.z &&
  a.radius === b.radius &&
  a.startAngle === b.startAngle &&
  a.endAngle === b.endAngle

function MapEditor() {
  const [history, setHistory] = useState<{
    past: EditorState[]
    present: EditorState
    future: EditorState[]
  }>(() => ({
    past: [],
    present: {
      lines: SAMPLE_LINES.map(cloneLineSegment),
      circles: SAMPLE_CIRCLES.map(cloneCircleArc),
    },
    future: [],
  }))
  const [selectedElement, setSelectedElement] =
    useState<SelectedElement | null>(null)
  const [status, setStatus] = useState<StatusMessage | null>(null)
  const mapViewRef = useRef<MapViewHandle>(null)
  const lines = history.present.lines
  const circles = history.present.circles
  const canUndo = history.past.length > 0
  const canRedo = history.future.length > 0

  const commitState = useCallback(
    (producer: (state: EditorState) => EditorState) => {
      setHistory((current) => {
        const next = producer(current.present)
        if (next === current.present) {
          return current
        }
        return {
          past: [...current.past, cloneEditorState(current.present)],
          present: next,
          future: [],
        }
      })
    },
    [],
  )

  const replacePresent = useCallback(
    (producer: (state: EditorState) => EditorState) => {
      setHistory((current) => {
        const next = producer(current.present)
        if (next === current.present) {
          return current
        }
        return {
          ...current,
          present: next,
        }
      })
    },
    [],
  )

  const beginHistoryInteraction = useCallback(() => {
    setHistory((current) => ({
      past: [...current.past, cloneEditorState(current.present)],
      present: current.present,
      future: [],
    }))
  }, [])

  const updateLineImmediate = useCallback(
    (lineId: string, updater: (line: LineSegment) => LineSegment) => {
      replacePresent((state) => {
        let didChange = false
        const nextLines = state.lines.map((line) => {
          if (line.id !== lineId) return line
          const updated = updater(line)
          if (updated === line || isLineEqual(updated, line)) {
            return line
          }
          didChange = true
          return cloneLineSegment(updated)
        })
        if (!didChange) return state
        return { ...state, lines: nextLines }
      })
    },
    [replacePresent],
  )

  const updateLineCommitted = useCallback(
    (lineId: string, updater: (line: LineSegment) => LineSegment) => {
      commitState((state) => {
        let didChange = false
        const nextLines = state.lines.map((line) => {
          if (line.id !== lineId) return line
          const updated = updater(line)
          if (updated === line || isLineEqual(updated, line)) {
            return line
          }
          didChange = true
          return cloneLineSegment(updated)
        })
        if (!didChange) return state
        return { ...state, lines: nextLines }
      })
    },
    [commitState],
  )

  const updateCircleImmediate = useCallback(
    (circleId: string, updater: (circle: CircleArc) => CircleArc) => {
      replacePresent((state) => {
        let didChange = false
        const nextCircles = state.circles.map((circle) => {
          if (circle.id !== circleId) return circle
          const updated = updater(circle)
          if (updated === circle || isCircleEqual(updated, circle)) {
            return circle
          }
          didChange = true
          return cloneCircleArc(updated)
        })
        if (!didChange) return state
        return { ...state, circles: nextCircles }
      })
    },
    [replacePresent],
  )

  const updateCircleCommitted = useCallback(
    (circleId: string, updater: (circle: CircleArc) => CircleArc) => {
      commitState((state) => {
        let didChange = false
        const nextCircles = state.circles.map((circle) => {
          if (circle.id !== circleId) return circle
          const updated = updater(circle)
          if (updated === circle || isCircleEqual(updated, circle)) {
            return circle
          }
          didChange = true
          return cloneCircleArc(updated)
        })
        if (!didChange) return state
        return { ...state, circles: nextCircles }
      })
    },
    [commitState],
  )

  const undo = useCallback(() => {
    setHistory((current) => {
      if (current.past.length === 0) return current
      const previous = current.past[current.past.length - 1]
      const newPast = current.past.slice(0, -1)
      return {
        past: newPast,
        present: previous,
        future: [cloneEditorState(current.present), ...current.future],
      }
    })
  }, [])

  const redo = useCallback(() => {
    setHistory((current) => {
      if (current.future.length === 0) return current
      const [next, ...rest] = current.future
      return {
        past: [...current.past, cloneEditorState(current.present)],
        present: next,
        future: rest,
      }
    })
  }, [])

  const selectedLine = useMemo(() => {
    if (selectedElement?.type !== 'line') return null
    return lines.find((line) => line.id === selectedElement.lineId) ?? null
  }, [lines, selectedElement])

  const selectedCircle = useMemo(() => {
    if (selectedElement?.type !== 'circle') return null
    return circles.find((circle) => circle.id === selectedElement.circleId) ?? null
  }, [circles, selectedElement])

  useEffect(() => {
    setSelectedElement((current) => {
      if (!current) return null
      if (
        (current.type === 'line' &&
          !lines.some((line) => line.id === current.lineId)) ||
        (current.type === 'circle' &&
          !circles.some((circle) => circle.id === current.circleId))
      ) {
        return null
      }
      return current
    })
  }, [lines, circles])

  const addLine = useCallback(() => {
    let createdLineId: string | null = null
    commitState((state) => {
      const last = state.lines[state.lines.length - 1]
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
      createdLineId = nextLine.id
      return { ...state, lines: [...state.lines, nextLine] }
    })
    if (createdLineId) {
      setSelectedElement({ type: 'line', lineId: createdLineId })
    }
  }, [commitState])

  const removeLine = useCallback(
    (lineId: string) => {
      commitState((state) => {
        const nextLines = state.lines.filter((line) => line.id !== lineId)
        if (nextLines.length === state.lines.length) return state
        return { ...state, lines: nextLines }
      })
      setSelectedElement((current) => {
        if (current?.type === 'line' && current.lineId === lineId) {
          return null
        }
        return current
      })
    },
    [commitState],
  )

  const addCircle = useCallback(() => {
    let createdCircleId: string | null = null
    commitState((state) => {
      const last = state.circles[state.circles.length - 1]
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
      createdCircleId = nextCircle.id
      return { ...state, circles: [...state.circles, nextCircle] }
    })
    if (createdCircleId) {
      setSelectedElement({ type: 'circle', circleId: createdCircleId })
    }
  }, [commitState])

  const removeCircle = useCallback(
    (circleId: string) => {
      commitState((state) => {
        const nextCircles = state.circles.filter(
          (circle) => circle.id !== circleId,
        )
        if (nextCircles.length === state.circles.length) return state
        return { ...state, circles: nextCircles }
      })
      setSelectedElement((current) => {
        if (current?.type === 'circle' && current.circleId === circleId) {
          return null
        }
        return current
      })
    },
    [commitState],
  )

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (!(event.ctrlKey || event.metaKey)) return
      const target = event.target as HTMLElement | null
      if (
        target &&
        (target.tagName === 'INPUT' ||
          target.tagName === 'TEXTAREA' ||
          target.tagName === 'SELECT' ||
          target.isContentEditable)
      ) {
        return
      }
      const key = event.key.toLowerCase()
      if (key === 'z' && event.shiftKey) {
        if (canRedo) {
          event.preventDefault()
          redo()
        }
        return
      }
      if (key === 'z' && !event.shiftKey) {
        if (canUndo) {
          event.preventDefault()
          undo()
        }
      }
    }

    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [undo, redo, canUndo, canRedo])

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

      if (hadLineSegmentFile || hadCircleFile) {
        commitState((state) => ({
          lines: hadLineSegmentFile
            ? newLines.map(cloneLineSegment)
            : state.lines,
          circles: hadCircleFile
            ? newCircles.map(cloneCircleArc)
            : state.circles,
        }))
        setSelectedElement(null)
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
            onBeginInteraction={beginHistoryInteraction}
            onSelectElement={setSelectedElement}
            onUpdateCircle={updateCircleImmediate}
            onUpdateLine={updateLineImmediate}
            selectedElement={selectedElement}
          />
          <div className="map-view__hud">
            <HudButton
              label="ズームイン"
              tooltip="ズームイン"
              onClick={() => mapViewRef.current?.zoomIn()}
            >
              <ZoomInIcon />
            </HudButton>
            <HudButton
              label="ズームアウト"
              tooltip="ズームアウト"
              onClick={() => mapViewRef.current?.zoomOut()}
            >
              <ZoomOutIcon />
            </HudButton>
            <HudButton
              label="フィット"
              tooltip="全体を表示"
              onClick={() => mapViewRef.current?.fitToContent()}
            >
              <FitViewIcon />
            </HudButton>
            <HudButton
              label="90°回転"
              tooltip="ビューを時計回りに90°回転"
              onClick={() => mapViewRef.current?.rotateClockwise()}
            >
              <RotateIcon />
            </HudButton>
            <HudButton
              label="回転リセット"
              tooltip="回転を初期化"
              onClick={() => mapViewRef.current?.resetRotation()}
            >
              <ResetRotationIcon />
            </HudButton>
          </div>
          <div className="map-view__history">
            <HistoryButton
              label="元に戻す"
              disabled={!canUndo}
              onClick={() => canUndo && undo()}
            >
              <UndoIcon />
            </HistoryButton>
            <HistoryButton
              label="やり直す"
              disabled={!canRedo}
              onClick={() => canRedo && redo()}
            >
              <RedoIcon />
            </HistoryButton>
          </div>
        </div>
        <InspectorPanel
          selectedCircle={selectedCircle}
          selectedLine={selectedLine}
          onUpdateCircle={updateCircleCommitted}
          onUpdateLine={updateLineCommitted}
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
  tooltip?: string
  onClick: () => void
  children: ReactNode
}

const HudButton = ({ label, tooltip = label, onClick, children }: HudButtonProps) => (
  <button
    type="button"
    className="map-view__hud-button"
    onClick={onClick}
    aria-label={label}
    data-tooltip={tooltip}
  >
    {children}
  </button>
)

type HistoryButtonProps = {
  label: string
  tooltip?: string
  disabled?: boolean
  onClick: () => void
  children: ReactNode
}

const HistoryButton = ({
  disabled = false,
  onClick,
  children,
}: HistoryButtonProps) => (
  <button
    type="button"
    className="map-view__history-button"
    onClick={onClick}
    disabled={disabled}
  >
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

const RotateIcon = () => (
  <svg viewBox="0 0 20 20" width="16" height="16" aria-hidden="true" focusable="false">
    <path
      d="M6.2 4.8a5.2 5.2 0 017.8 3.7"
      stroke="currentColor"
      strokeWidth="1.6"
      fill="none"
      strokeLinecap="round"
    />
    <polyline
      points="12.8 4.4 15 6.4 12.8 8.4"
      stroke="currentColor"
      strokeWidth="1.6"
      fill="none"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
    <path
      d="M5 15.2a5.2 5.2 0 01-1-7.2"
      stroke="currentColor"
      strokeWidth="1.6"
      fill="none"
      strokeLinecap="round"
    />
  </svg>
)

const ResetRotationIcon = () => (
  <svg viewBox="0 0 20 20" width="16" height="16" aria-hidden="true" focusable="false">
    <circle cx="10" cy="10" r="6.2" stroke="currentColor" strokeWidth="1.4" fill="none" opacity="0.85" />
    <line
      x1="10"
      y1="5.2"
      x2="10"
      y2="14.8"
      stroke="currentColor"
      strokeWidth="1.4"
      strokeLinecap="round"
    />
    <line
      x1="5.2"
      y1="10"
      x2="14.8"
      y2="10"
      stroke="currentColor"
      strokeWidth="1.4"
      strokeLinecap="round"
    />
  </svg>
)

const UndoIcon = () => (
  <svg viewBox="0 0 20 20" width="16" height="16" aria-hidden="true" focusable="false">
    <polyline
      points="9.2 4.8 4.8 9.6 9.2 14.4"
      stroke="currentColor"
      strokeWidth="1.6"
      fill="none"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
    <path
      d="M5.6 9.6h6.4a4.4 4.4 0 010 8.8H11"
      stroke="currentColor"
      strokeWidth="1.6"
      fill="none"
      strokeLinecap="round"
    />
  </svg>
)

const RedoIcon = () => (
  <svg viewBox="0 0 20 20" width="16" height="16" aria-hidden="true" focusable="false">
    <polyline
      points="10.8 4.8 15.2 9.6 10.8 14.4"
      stroke="currentColor"
      strokeWidth="1.6"
      fill="none"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
    <path
      d="M14.4 9.6H8a4.4 4.4 0 000 8.8h1"
      stroke="currentColor"
      strokeWidth="1.6"
      fill="none"
      strokeLinecap="round"
    />
  </svg>
)

export default MapEditor
