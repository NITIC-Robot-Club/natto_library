import type { CircleArc, LineSegment, SelectedElement, StatusMessage } from '../types'

type GeometryListPanelProps = {
  lines: LineSegment[]
  circles: CircleArc[]
  selectedElement: SelectedElement | null
  status: StatusMessage | null
  onStatusClear: () => void
  onAddLine: () => void
  onRemoveLine: (lineId: string) => void
  onAddCircle: () => void
  onRemoveCircle: (circleId: string) => void
  onSelectElement: (element: SelectedElement | null) => void
}

export const GeometryListPanel = ({
  lines,
  circles,
  selectedElement,
  status,
  onStatusClear,
  onAddLine,
  onRemoveLine,
  onAddCircle,
  onRemoveCircle,
  onSelectElement,
}: GeometryListPanelProps) => {
  return (
    <aside className="inspector inspector--lists">
      <header className="inspector__header">
        <p className="inspector__title">Elements</p>
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
        <div className="inspector__section-header">
          <h3 className="inspector__section-title">Lines</h3>
          <button
            type="button"
            className="inspector__add-button"
            onClick={onAddLine}
          >
            + Add line
          </button>
        </div>
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
      </section>
      <section className="inspector__section">
        <div className="inspector__section-header">
          <h3 className="inspector__section-title">Circles</h3>
          <button
            type="button"
            className="inspector__add-button"
            onClick={onAddCircle}
          >
            + Add circle
          </button>
        </div>
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
      </section>
    </aside>
  )
}
