/** The last project state, kept in the browser so a page reload doesn't lose unsaved work. IndexedDB, not
 *  localStorage: a robot project is easily 10+ MB, past localStorage's limit. Every call is best effort. */
export interface Saved { text: string; at: number }
export interface SaveStore { read(): Promise<Saved | null>; write(s: Saved): Promise<void>; clear(): Promise<void> }

const DB = "cad-to-urdf", STORE = "autosave", KEY = "project";

function db(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const r = indexedDB.open(DB, 1);
    r.onupgradeneeded = () => r.result.createObjectStore(STORE);
    r.onsuccess = () => resolve(r.result);
    r.onerror = () => reject(r.error);
  });
}

async function tx<T>(mode: IDBTransactionMode, run: (s: IDBObjectStore) => IDBRequest): Promise<T | null> {
  try {
    const d = await db();
    return await new Promise<T | null>((resolve, reject) => {
      const r = run(d.transaction(STORE, mode).objectStore(STORE));
      r.onsuccess = () => resolve((r.result as T) ?? null);
      r.onerror = () => reject(r.error);
    });
  } catch {
    return null;   // private window, blocked storage, no IndexedDB (tests): no autosave
  }
}

export const browserStore: SaveStore = {
  read: () => tx<Saved>("readonly", (s) => s.get(KEY)),
  write: async (v) => { await tx("readwrite", (s) => s.put(v, KEY)); },
  clear: async () => { await tx("readwrite", (s) => s.delete(KEY)); },
};

export function memoryStore(initial: Saved | null = null): SaveStore & { saved: Saved | null } {
  const m = {
    saved: initial,
    read: async () => m.saved,
    write: async (v: Saved) => { m.saved = v; },
    clear: async () => { m.saved = null; },
  };
  return m;
}
