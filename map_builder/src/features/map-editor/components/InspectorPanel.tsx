import type { ChangeEvent } from 'react'
import type { CircleArc, LineSegment, SelectedElement } from '../types'

type StatusMessage = {
  tone: 'info' | 'error' | 'success'
  text: string
}

type InspectorPanelProps = {
  selectedElement: SelectedElement | null
  selectedLine: LineSegment | null
  selectedCircle: CircleArc | null
  lines: LineSegment[]
  circles: CircleArc[]
  status: StatusMessage | null
  onStatusClear: () => void
  onAddLine: () => void
  onRemoveLine: (lineId: string) => void
  onAddCircle: () => void
  onRemoveCircle: (circleId: string) => void
  onUpdateLine: (
    lineId: string,
    updater: (line: LineSegment) => LineSegment,
  ) => void
  onUpdateCircle: (
    circleId: string,
    updater: (circle: CircleArc) => CircleArc,
  ) => void
  onSelectElement: (element: SelectedElement | null) => void
}

export const InspectorPanel = ({
  lines,
  circles,
  selectedElement,
  selectedLine,
  selectedCircle,
  status,
  onStatusClear,
  onAddLine,
  onRemoveLine,
  onAddCircle,
  onRemoveCircle,
  onUpdateCircle,
  onUpdateLine,
  onSelectElement,
}: InspectorPanelProps) => {
  const updateLineEndpointCoordinate = (
    lineId: string,
    endpoint: 'start' | 'end',
    axis: 'x' | 'y' | 'z',
    value: number,
  ) => {
    onUpdateLine(lineId, (line) => ({
      ...line,
      [endpoint]: {
        ...line[endpoint],
        [axis]: value,
      },
    }))
  }

  const handleNumberInput = (
    event: ChangeEvent<HTMLInputElement>,
    onCommit: (value: number) => void,
  ) => {
    const value = parseFloat(event.target.value)
    if (!Number.isNaN(value)) {
      onCommit(value)
    }
  }

  return (
    <aside className="inspector">
      <header className="inspector__header">
        <div>
          <p className="inspector__title">Inspector</p>
        </div>
      </header>
      {status ? (
        <div
          className={`inspector__status inspector__status--${status.tone}`}
          role="status"
        >
          <span>{status.text}</span>
          <button
            type="button"
            className="inspector__status-dismiss"
            onClick={onStatusClear}
            aria-label="閉じる"
          >
            ×
          </button>
        </div>
      ) : null}
      {selectedLine ? (
        <div className="inspector__properties">
          <h3 className="inspector__section-title">Line Properties</h3>
          <span className="inspector__label-heading">Start</span>
          <div className="inspector__form-grid">
            {(['x', 'y', 'z'] as const).map((axis) => (
              <label key={axis} className="inspector__label">
                {axis.toUpperCase()}
                <input
                  type="number"
                  step="0.01"
                  value={selectedLine.start[axis]}
                  onChange={(event) =>
                    handleNumberInput(event, (value) =>
                      updateLineEndpointCoordinate(
                        selectedLine.id,
                        'start',
                        axis,
                        value,
                      ),
                    )
                  }
                />
              </label>
            ))}
          </div>
          <span className="inspector__label-heading">End</span>
          <div className="inspector__form-grid">
            {(['x', 'y', 'z'] as const).map((axis) => (
              <label key={axis} className="inspector__label">
                {axis.toUpperCase()}
                <input
                  type="number"
                  step="0.01"
                  value={selectedLine.end[axis]}
                  onChange={(event) =>
                    handleNumberInput(event, (value) =>
                      updateLineEndpointCoordinate(
                        selectedLine.id,
                        'end',
                        axis,
                        value,
                      ),
                    )
                  }
                />
              </label>
            ))}
          </div>
        </div>
      ) : selectedCircle ? (
        <div className="inspector__properties">
          <h3 className="inspector__section-title">Circle Properties</h3>
          <span className="inspector__label-heading">Center</span>
          <div className="inspector__form-grid">
            {(['x', 'y', 'z'] as const).map((axis) => (
              <label key={axis} className="inspector__label">
                {axis.toUpperCase()}
                <input
                  type="number"
                  step="0.01"
                  value={selectedCircle.center[axis]}
                  onChange={(event) =>
                    handleNumberInput(event, (value) =>
                      onUpdateCircle(selectedCircle.id, (circle) => ({
                        ...circle,
                        center: {
                          ...circle.center,
                          [axis]: value,
                        },
                      })),
                    )
                  }
                />
              </label>
            ))}
          </div>
          <div className="inspector__form-grid inspector__form-grid--dense">
            <label className="inspector__label">
              Radius (m)
              <input
                type="number"
                step="0.01"
                min="0"
                value={selectedCircle.radius}
                onChange={(event) =>
                  handleNumberInput(event, (value) =>
                    onUpdateCircle(selectedCircle.id, (circle) => ({
                      ...circle,
                      radius: Math.max(value, 0),
                    })),
                  )
                }
              />
            </label>
            <label className="inspector__label">
              Start angle (rad)
              <input
                type="number"
                step="0.1"
                value={selectedCircle.startAngle}
                onChange={(event) =>
                  handleNumberInput(event, (value) =>
                    onUpdateCircle(selectedCircle.id, (circle) => ({
                      ...circle,
                      startAngle: value,
                    })),
                  )
                }
              />
            </label>
            <label className="inspector__label">
              End angle (rad)
              <input
                type="number"
                step="0.1"
                value={selectedCircle.endAngle}
                onChange={(event) =>
                  handleNumberInput(event, (value) =>
                    onUpdateCircle(selectedCircle.id, (circle) => ({
                      ...circle,
                      endAngle: value,
                    })),
                  )
                }
              />
            </label>
          </div>
        </div>
      ) : (
        <p className="inspector__hint">MapViewまたはラインリストから要素を選択してください。</p>
      )}
      <section className="inspector__section">
        <h3 className="inspector__section-title">Lines</h3>
        {lines.length === 0 ? (
          <p className="inspector__empty">なし</p>
        ) : (
          <div className="inspector__list">
            {lines.map((line, index) => {
              const isActive =
                selectedElement?.type === 'line' &&
                selectedElement.lineId === line.id
              return (
                <div
                  key={line.id}
                  className={`inspector__list-item ${isActive ? 'is-active' : ''}`}
                >
                  <button
                    type="button"
                    className="inspector__list-trigger"
                    onClick={() =>
                      onSelectElement({ type: 'line', lineId: line.id })
                    }
                  >
                    <span className="inspector__list-label">#{index + 1}</span>
                    <span className="inspector__list-meta">
                      ({line.start.x.toFixed(2)}, {line.start.y.toFixed(2)}) -&gt; ({line.end.x.toFixed(2)}, {line.end.y.toFixed(2)})
                    </span>
                  </button>
                  <div className="inspector__list-actions">
                    <button
                      type="button"
                      className="inspector__icon-button inspector__icon-button--danger"
                      onClick={(event) => {
                        event.stopPropagation()
                        onRemoveLine(line.id)
                      }}
                      aria-label="線を削除"
                    >
                      ×
                    </button>
                  </div>
                </div>
              )
            })}
          </div>
        )}
        <button type="button" className="inspector__add-button" onClick={onAddLine}>
          + Add line
        </button>
      </section>
      <section className="inspector__section">
        <h3 className="inspector__section-title">Circles</h3>
        {circles.length === 0 ? (
          <p className="inspector__empty">なし</p>
        ) : (
          <div className="inspector__list">
            {circles.map((circle, index) => {
              const isActive =
                selectedElement?.type === 'circle' &&
                selectedElement.circleId === circle.id
              return (
                <div
                  key={circle.id}
                  className={`inspector__list-item ${isActive ? 'is-active' : ''}`}
                >
                  <button
                    type="button"
                    className="inspector__list-trigger"
                    onClick={() =>
                      onSelectElement({ type: 'circle', circleId: circle.id })
                    }
                  >
                    <span className="inspector__list-label">#{index + 1}</span>
                    <span className="inspector__list-meta">
                      ({circle.center.x.toFixed(2)}, {circle.center.y.toFixed(2)})
                    </span>
                  </button>
                  <div className="inspector__list-actions">
                    <button
                      type="button"
                      className="inspector__icon-button inspector__icon-button--danger"
                      onClick={(event) => {
                        event.stopPropagation()
                        onRemoveCircle(circle.id)
                      }}
                      aria-label="サークルを削除"
                    >
                      ×
                    </button>
                  </div>
                </div>
              )
            })}
          </div>
        )}
        <button type="button" className="inspector__add-button" onClick={onAddCircle}>
          + Add circle
        </button>
      </section>
    </aside>
  )
}
