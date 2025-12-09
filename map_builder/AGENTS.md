# MapBuilder Guidelines

## Project Structure & Module Organization
- `src/` holds the React + TypeScript application; entry flow starts at `main.tsx` and renders `App.tsx`. Keep new components in feature folders under `src/` with colocated styles when helpful.
- `src/assets/` is for static assets bundled by Vite; larger shared media should move to `public/` so it stays cacheable at the root.
- Tooling lives at the root: `vite.config.ts`, `tsconfig*.json`, and `eslint.config.js`. Adjust build-time behavior there rather than in component files.

## Build, Test, and Development Commands
- `npm install` — install dependencies; rerun after updating `package.json`.
- `npm run dev` — start the Vite dev server with hot module reload at `http://localhost:5173`.
- `npm run build` — type-check with `tsc -b` and emit an optimized production bundle in `dist/`.
- `npm run preview` — serve the built bundle to validate production output.
- `npm run lint` — run ESLint against the repo; resolve warnings before opening a PR.

## Coding Style & Naming Conventions
- TypeScript is in strict mode; favor explicit `type`/`interface` declarations and narrow React component props.
- Use 2-space indentation, single quotes for strings, and trailing commas in multiline literals to match existing files.
- Name React components and hooks with `PascalCase` (`MapPanel`) and `camelCase` (`useViewport`) respectively; keep filenames aligned with their default export.
- ESLint (extending `@eslint/js`, `typescript-eslint`, and React rules) is the source of truth—run `npm run lint` after refactors.

## Testing Guidelines
- No automated tests exist yet; when adding features, create colocated files named `*.test.tsx` using Vitest + React Testing Library (recommended stack) and include interaction coverage for map behaviors.
- Keep tests deterministic and avoid relying on network calls; stub asynchronous map services.
- Document any manual verification steps in your PR when automated coverage is not feasible.

## Commit & Pull Request Guidelines
- Git history is not yet established—use Conventional Commits (`feat: add tile layer controls`, `fix: correct zoom reset`) to keep logs readable.
- Each PR should include: a concise summary, testing notes (`npm run lint`, manual steps), linked issue IDs, and screenshots or GIFs for UI changes.
- Rebase before merging to maintain a linear history; avoid merge commits unless coordinating large feature branches.

---

# File Structure & Responsibilities

## Root
- `AGENTS.md` — working notes for AI agents; includes contribution guidelines and this structure digest.
- `README.md` — developer-facing quick start (currently minimal).
- `index.html` — Vite entry document that mounts React into `#root` and pulls `/src/main.tsx`.
- `package.json` / `package-lock.json` — project metadata, scripts, and dependency pins (React 19 + Vite 7 toolchain).
- `vite.config.ts` — Vite setup enabling the React plugin; extend here for build tooling tweaks.
- `eslint.config.js` — flat ESLint config combining `@eslint/js`, `typescript-eslint`, React Hooks, and React Refresh rules; ignores `dist/`.
- `tsconfig.json` — composite config referencing app and node builds.
- `tsconfig.app.json` — strict TS settings for client code (bundler resolution, React JSX transform, no emit).
- `tsconfig.node.json` — TS settings for tooling files like `vite.config.ts`.
- `dist/` — build output created by `npm run build`; excluded from linting and source control review.
- `node_modules/` — installed dependencies and TypeScript build info caches.

## Application Entry (`src/`)
- `src/main.tsx` — bootstraps React 19 with `StrictMode`, imports global styles, and renders `<App />` into the DOM.
- `src/App.tsx` — root component delegating to the map editor feature; simple composition point for future routing/layout.
- `src/index.css` — global styles: sets dark theme palette, typography, resets, and ensures `body`/`#root` fill the viewport.
- `src/App.css` — legacy Vite starter styles (logo animation, card styles); currently unused but retained as a reference.

## Map Editor Feature (`src/features/map-editor`)
- `MapEditor.tsx` — primary feature orchestrator. Holds sample geometry data in state, wires CSV import/export, tracks selections, exposes zoom controls through a `MapView` ref, and renders the layout (`TopBar`, `InspectorPanel`, `MapView`, HUD buttons). Also houses CSV parsing helpers, ID generation, and inline HUD icon components.
- `mapEditor.css` — detailed layout and visual styling for the editor shell, inspector, map view, HUD controls, and interactive handles.
- `types.ts` — shared TypeScript types for vectors, points, circles, selection state, and viewport metadata.
- `utils.ts` — geometry helpers: derives line segments from an ordered point list (closing the loop) and compares 3D vectors with tolerance.

### Feature Components (`src/features/map-editor/components`)
- `MapView.tsx` — interactive SVG canvas. Renders segments, points, and circle arcs; manages viewport pan/zoom with wheel and pointer input; exposes imperative `fitToContent/zoom/reset` methods; handles drag interactions for points and circle handles; draws a grid background.
- `InspectorPanel.tsx` — sidebar listing points and circles, provides numeric inputs to edit coordinates/radii/angles, displays async status messages, and drives selection changes.
- `TopBar.tsx` — header with branding, upload/download controls, and hidden file input that triggers CSV import.

## Utilities
- CSV helpers in `MapEditor.tsx` expect `line_segments.csv` and `circles.csv` headers, rebuild the point loop while avoiding duplicate endpoints, and surface localized status messages (`success`, `error`, `info`).
- HUD button icons (`ZoomInIcon`, `ZoomOutIcon`, `FitViewIcon`) are inline SVG components in `MapEditor.tsx` to keep the HUD self-contained.

---

# Operational Notes
- Run `npm run dev` for local development; `npm run build` performs strict type checking before bundling.
- ESLint is configured via `npm run lint`; resolve warnings before committing.
- Prefer colocating new feature code under `src/features/<feature-name>` with relevant styles and tests.
