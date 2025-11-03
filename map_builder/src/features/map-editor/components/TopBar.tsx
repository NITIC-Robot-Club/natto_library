import { useRef } from 'react'

type TopBarProps = {
  onRequestFileLoad: (files: FileList | null) => Promise<void> | void
  onDownload: () => void
}

export const TopBar = ({
  onRequestFileLoad,
  onDownload,
}: TopBarProps) => {
  const fileInputRef = useRef<HTMLInputElement | null>(null)

  return (
    <div className="map-editor__top-bar">
      <div className="map-editor__brand">
        <span className="map-editor__brand-mark" />
        <div>
          <p className="map-editor__brand-title">Map Builder</p>
          <p className="map-editor__brand-subtitle">Line & Circle Editor</p>
        </div>
      </div>
      <div className="map-editor__controls">
        <div className="map-editor__controls-group">
          <button
            type="button"
            className="map-editor__controls-button"
            onClick={() => fileInputRef.current?.click()}
          >
            ファイルから読み込み
          </button>
          <button
            type="button"
            className="map-editor__controls-button map-editor__controls-button--primary"
            onClick={onDownload}
          >
            ダウンロード
          </button>
          <input
            ref={fileInputRef}
            type="file"
            accept=".csv"
            multiple
            hidden
            onChange={async (event) => {
              await onRequestFileLoad(event.target.files)
              event.target.value = ''
            }}
          />
        </div>
      </div>
    </div>
  )
}
