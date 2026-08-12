import { useEffect, useRef, useState, type RefObject } from 'react'
import { flushSync } from 'react-dom'
import { createRoot } from 'react-dom/client'
import {
  AppShell,
  Button,
  CodeBlock,
  Panel,
  ProductBrand,
  ResourceMeter,
  ScrollRegion,
  SegmentedControl,
  StatusText,
  Topbar,
} from '@xgc2/ui-react'
import '@xgc2/ui-react/styles.css'
import './styles.css'

type Skin = 'light' | 'dark'

function useLegacyMutation(ref: RefObject<HTMLElement | null>, sync: () => void) {
  useEffect(() => {
    const node = ref.current
    if (!node) return
    sync()
    const observer = new MutationObserver(sync)
    observer.observe(node, { attributes: true, childList: true, subtree: true })
    return () => observer.disconnect()
  }, [])
}

function LegacyStatus({ id, initial = '', hideValues = [] }: { id: string; initial?: string; hideValues?: string[] }) {
  const ref = useRef<HTMLSpanElement>(null)
  const [value, setValue] = useState(initial)
  const sync = () => setValue(ref.current?.textContent || '')
  useLegacyMutation(ref, sync)
  const normalized = value.trim().toLowerCase()
  const tone = /disconnect|failed|error/.test(normalized) ? 'danger' : /connect|wait/.test(normalized) ? 'info' : 'neutral'
  return (
    <>
      <span ref={ref} id={id} className="legacy-state-source pill pill-off">{initial}</span>
      {hideValues.includes(normalized) ? null : <StatusText status={value || 'idle'} tone={tone}>{value}</StatusText>}
    </>
  )
}

function LegacyCodeResult() {
  const ref = useRef<HTMLPreElement>(null)
  const [snapshot, setSnapshot] = useState({ visible: false, value: '' })
  const sync = () => {
    const node = ref.current
    if (node) setSnapshot({ visible: !node.hidden && Boolean(node.textContent), value: node.textContent || '' })
  }
  useLegacyMutation(ref, sync)
  return (
    <>
      <pre ref={ref} id="result" className="legacy-state-source" hidden />
      {snapshot.visible ? <CodeBlock className="gazebo-result" content={snapshot.value} label="Calibration result" language="text" /> : null}
    </>
  )
}

type Meter = { label: string; percent: number }

function LegacyCoverage() {
  const ref = useRef<HTMLDivElement>(null)
  const [meters, setMeters] = useState<Meter[]>([])
  const sync = () => {
    const node = ref.current
    if (!node) return
    setMeters(Array.from(node.querySelectorAll<HTMLElement>('.bar')).map((bar) => ({
      label: bar.querySelector('.bar-label span')?.textContent || 'Coverage',
      percent: Number.parseFloat(bar.querySelector('.pct')?.textContent || '0') || 0,
    })))
  }
  useLegacyMutation(ref, sync)
  return (
    <>
      <div ref={ref} id="bars" className="legacy-state-source" />
      <div className="gazebo-meters">
        {meters.map((meter) => (
          <ResourceMeter
            key={meter.label}
            label={meter.label}
            detail={`${meter.percent}%`}
            percent={meter.percent}
            tone={meter.percent >= 100 ? 'success' : 'warning'}
          />
        ))}
      </div>
    </>
  )
}

function ThemeControl() {
  const [skin, setSkin] = useState<Skin>(readSkin)
  useEffect(() => {
    document.documentElement.dataset.skin = skin
    try { localStorage.setItem('xgc2-gazebo-camera.skin', skin) } catch { /* storage may be unavailable */ }
  }, [skin])
  return (
    <SegmentedControl
      ariaLabel="Appearance"
      value={skin}
      options={[{ label: 'Light', value: 'light' }, { label: 'Dark', value: 'dark' }]}
      onValueChange={(value) => setSkin(value as Skin)}
    />
  )
}

function App() {
  return (
    <AppShell
      className="gazebo-shell"
      contentClassName="gazebo-content"
      contentPadding="none"
      mobileLayout="document"
      topbar={<Topbar brand={<ProductBrand product="Gazebo camera calibration" />} actions={<ThemeControl />} />}
    >
      <main className="gazebo-page">
        <Panel
          className="gazebo-view-panel"
          padding="none"
          title="Live camera"
          actions={<LegacyStatus id="conn" initial="connecting…" hideValues={['connected']} />}
        >
          <div className="gazebo-frame"><img id="stream" src="/stream.mjpg" alt="Camera stream" /></div>
          <p className="gazebo-hint">Live camera view. Select a sphere in the guide to move the simulated camera, or use Auto-run to sweep every pose and validate coverage.</p>
        </Panel>

        <ScrollRegion className="gazebo-side" fill>
          <Panel title="Coverage" actions={<span id="samples" className="gazebo-meta">0 samples</span>}>
            <LegacyCoverage />
          </Panel>

          <Panel id="calib-card" title="Calibration">
            <div className="gazebo-actions">
              <Button id="btn-calibrate" disabled>Calibrate</Button>
              <Button id="btn-save" disabled>Save</Button>
              <Button id="btn-commit" tone="primary" appearance="solid" disabled>Commit</Button>
            </div>
            <LegacyStatus id="status" hideValues={['']} />
            <LegacyCodeResult />
          </Panel>

          <Panel id="camera-card" title="Sample guide">
            <p className="gazebo-hint">Drag to rotate the guide; select a sphere to move the camera.</p>
            <canvas id="scene" />
            <div className="gazebo-next">
              <p>Next: <strong id="next-name">—</strong><span id="done-count" className="gazebo-meta" /></p>
              <img id="ref-img" alt="Expected view" hidden />
              <p id="ref-hint" className="gazebo-meta">Select the next target; captured poses are marked in the guide.</p>
            </div>
            <div className="gazebo-actions">
              <Button id="btn-reset">Reset pose</Button>
              <Button id="btn-auto">Auto-run</Button>
            </div>
            <p id="pose" className="gazebo-meta gazebo-pose" />
          </Panel>
        </ScrollRegion>
      </main>
    </AppShell>
  )
}

function readSkin(): Skin {
  try { return localStorage.getItem('xgc2-gazebo-camera.skin') === 'light' ? 'light' : 'dark' } catch { return 'dark' }
}

const root = document.getElementById('app')
if (!root) throw new Error('Gazebo camera calibration root is unavailable')
flushSync(() => createRoot(root).render(<App />))
void import('./legacy')
