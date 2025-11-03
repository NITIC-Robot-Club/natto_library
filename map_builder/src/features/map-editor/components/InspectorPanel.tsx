import type { ChangeEvent } from 'react'
import type { CircleArc, LineSegment } from '../types'

type InspectorPanelProps = {
  selectedLine: LineSegment | null
  selectedCircle: CircleArc | null
  onUpdateLine: (
    lineId: string,
    updater: (line: LineSegment) => LineSegment,
  ) => void
  onUpdateCircle: (
    circleId: string,
    updater: (circle: CircleArc) => CircleArc,
  ) => void
}

export const InspectorPanel = ({
  selectedLine,
  selectedCircle,
  onUpdateCircle,
  onUpdateLine,
}: InspectorPanelProps) => {
  const radToDeg = (rad: number) => (rad * 180) / Math.PI
  const degToRad = (deg: number) => (deg * Math.PI) / 180

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
    <aside className="inspector inspector--properties">
      <header className="inspector__header">
        <p className="inspector__title">Properties</p>
      </header>
      {selectedLine ? (
        <section className="inspector__properties">
          <h3 className="inspector__section-title">Line</h3>
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
        </section>
      ) : selectedCircle ? (
        <section className="inspector__properties">
          <h3 className="inspector__section-title">Circle</h3>
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
              Start angle (deg)
              <input
                type="number"
                step="1"
                value={radToDeg(selectedCircle.startAngle)}
                onChange={(event) =>
                  handleNumberInput(event, (value) =>
                    onUpdateCircle(selectedCircle.id, (circle) => ({
                      ...circle,
                      startAngle: degToRad(value),
                    })),
                  )
                }
              />
            </label>
            <label className="inspector__label">
              End angle (deg)
              <input
                type="number"
                step="1"
                value={radToDeg(selectedCircle.endAngle)}
                onChange={(event) =>
                  handleNumberInput(event, (value) =>
                    onUpdateCircle(selectedCircle.id, (circle) => ({
                      ...circle,
                      endAngle: degToRad(value),
                    })),
                  )
                }
              />
            </label>
          </div>
        </section>
      ) : (
        <p className="inspector__hint inspector__hint--centered">
          MapViewまたは左のリストから要素を選択してください。
        </p>
      )}
    </aside>
  )
}
