export type Hud = {
  /** What the owner last said with /kilo up or /kilo down. */
  owner: 'up' | 'down' | 'unknown'
  ownerAt: number
  /** Whether the last name lookup (kilo find) got an answer from Kilo. */
  isFound: boolean
  address: string
  mindBuild: string
  checkedAt: number
  /** From the last full status read (only on appearing, /kilo, /kilo up, /kilo refresh). */
  mode: string
  batteryMv: number
  firmware: string
  statusAt: number
  statusError: string
  /** The repository's HEAD commit, 8 hex digits. */
  repoHead: string
}

declare module 'claude-code' {
  interface PluginState {
    'kilo-hud': { hud: Shaped<Hud> }
  }
}
