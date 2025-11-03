import type { ChangeEvent } from 'react'
import type { CircleArc, MapPoint, SelectedElement } from '../types'

type StatusMessage = {
  tone: 'info' | 'error' | 'success'
  text: string
}

type InspectorPanelProps = {
  selectedElement: SelectedElement | null
  selectedPoint: MapPoint | null
  selectedCircle: CircleArc | null
  points: MapPoint[]
  circles: CircleArc[]
  status: StatusMessage | null
  onStatusClear: () => void
  onAddPoint: () => void
  onRemovePoint: (pointId: string) => void
  onReorderPoint: (pointId: string, targetIndex: number) => void
  onAddCircle: () => void
  onRemoveCircle: (circleId: string) => void
  onUpdatePoint: (
    pointId: string,
    updater: (point: MapPoint) => MapPoint,
  ) => void
  onUpdateCircle: (
    circleId: string,
    updater: (circle: CircleArc) => CircleArc,
  ) => void
  onSelectElement: (element: SelectedElement | null) => void
}

export const InspectorPanel = ({
  points,
  circles,
  selectedElement,
  selectedPoint,
  selectedCircle,
  status,
  onStatusClear,
  onAddPoint,
  onRemovePoint,
  onReorderPoint,
  onAddCircle,
  onRemoveCircle,
  onUpdateCircle,
  onUpdatePoint,
  onSelectElement,
}: InspectorPanelProps) => {
  const updatePointCoordinate = (
    pointId: string,
    axis: 'x' | 'y' | 'z',
    value: number,
  ) => {
    onUpdatePoint(pointId, (point) => ({
      ...point,
      position: {
        ...point.position,
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
      <section className="inspector__section">
        <h3 className="inspector__section-title">Points</h3>
        {points.length === 0 ? (
          <p className="inspector__empty">なし</p>
        ) : (
          <div className="inspector__list">
            {points.map((point, index) => {
              const isActive =
                selectedElement?.type === 'point' &&
                selectedElement.pointId === point.id
              return (
                <div
                  key={point.id}
                  className={`inspector__list-item ${isActive ? 'is-active' : ''}`}
                >
                  <button
                    type="button"
                    className="inspector__list-trigger"
                    onClick={() =>
                      onSelectElement({ type: 'point', pointId: point.id })
                    }
                  >
                    <span className="inspector__list-label">#{index + 1}</span>
                    <span className="inspector__list-meta">
                      ({point.position.x.toFixed(2)}, {point.position.y.toFixed(2)})
                    </span>
                  </button>
                  <div className="inspector__list-actions">
                    <button
                      type="button"
                      className="inspector__icon-button"
                      onClick={(event) => {
                        event.stopPropagation()
                        onReorderPoint(point.id, index - 1)
                      }}
                      disabled={index === 0}
                      aria-label="上に移動"
                    >
                      ↑
                    </button>
                    <button
                      type="button"
                      className="inspector__icon-button"
                      onClick={(event) => {
                        event.stopPropagation()
                        onReorderPoint(point.id, index + 1)
                      }}
                      disabled={index === points.length - 1}
                      aria-label="下に移動"
                    >
                      ↓
                    </button>
                    <button
                      type="button"
                      className="inspector__icon-button inspector__icon-button--danger"
                      onClick={(event) => {
                        event.stopPropagation()
                        onRemovePoint(point.id)
                      }}
                      aria-label="ポイントを削除"
                    >
                      ×
                    </button>
                  </div>
                </div>
              )
            })}
          </div>
        )}
        <button type="button" className="inspector__add-button" onClick={onAddPoint}>
          + Add point
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
      {selectedPoint ? (
        <div className="inspector__properties">
          <h3 className="inspector__section-title">Point Properties</h3>
          <div className="inspector__form-grid">
            {(['x', 'y', 'z'] as const).map((axis) => (
              <label key={axis} className="inspector__label">
                {axis.toUpperCase()}
                <input
                  type="number"
                  step="0.01"
                  value={selectedPoint.position[axis]}
                  onChange={(event) =>
                    handleNumberInput(event, (value) =>
                      updatePointCoordinate(selectedPoint.id, axis, value),
                    )
                  }
                />
              </label>
            ))}
          </div>
        </div>
      ) : null}
      {selectedCircle ? (
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
      ) : null}
      {!selectedPoint && !selectedCircle ? (
        <p className="inspector__hint">MapViewまたはポイントリストから要素を選択してください。</p>
      ) : null}
    </aside>
  )
}
