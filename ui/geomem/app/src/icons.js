const paths = {
  plus:'M12 5v14 M5 12h14',
  minus:'M5 12h14',
  close:'m6 6 12 12 M6 18 18 6',
 layout:'M3 3h7v7H3z M14 3h7v7h-7z M3 14h7v7H3z M14 14h7v7h-7z',
 box:'m12 3 9 5-9 5-9-5 9-5z M3 8v9l9 5 9-5V8 M12 13v9 M7 5l10 6',
 history:'M3 11a9 9 0 1 1 2 7 M3 4v7h7 M12 7v5l3 2',
 robot:'M5 7h14a2 2 0 0 1 2 2v9H3V9a2 2 0 0 1 2-2z M12 3v4 M8 12h.01 M16 12h.01 M8 16h8 M1 11v4 M23 11v4 M6 18v3 M18 18v3',
 help:'M12 22a10 10 0 1 0 0-20 10 10 0 0 0 0 20z M9 8a3 3 0 0 1 6 0c0 2-3 2-3 5 M12 17h.01',
 sliders:'M4 3v7 M4 14v7 M12 3v12 M12 19v2 M20 3v2 M20 9v12 M1 10h6v4H1z M9 15h6v4H9z M17 5h6v4h-6z',
 expand:'M8 3H3v5 M16 3h5v5 M3 16v5h5 M16 21h5v-5',
 focus:'M8 3H3v5 M16 3h5v5 M3 16v5h5 M16 21h5v-5 M12 8a4 4 0 1 0 0 8 4 4 0 0 0 0-8z',
 layers:'m12 3 10 5-10 5L2 8l10-5z M2 12l10 5 10-5 M2 16l10 5 10-5',
 sparkle:'m12 2 2.5 7.5L22 12l-7.5 2.5L12 22l-2.5-7.5L2 12l7.5-2.5L12 2z',
 search:'M10.5 3a7.5 7.5 0 1 0 0 15 7.5 7.5 0 0 0 0-15z M16 16l5 5',
 arrow:'M4 12h16 M14 6l6 6-6 6',
 locate:'M12 4a8 8 0 1 0 0 16 8 8 0 0 0 0-16z M12 1v5 M12 18v5 M1 12h5 M18 12h5 M12 9a3 3 0 1 0 0 6 3 3 0 0 0 0-6z',
 shield:'M12 2 3 6v6c0 6 9 10 9 10s9-4 9-10V6l-9-4z m-5 10 3 3 6-6',
 pin:'M12 22s8-8 8-14A8 8 0 0 0 4 8c0 6 8 14 8 14z M12 5a3 3 0 1 0 0 6 3 3 0 0 0 0-6z',
 activity:'M2 12h5l3-8 4 16 3-8h5',
 download:'M12 3v12 m-5-5 5 5 5-5 M3 16v5h18v-5',
 backpack:'M8 6V4a4 4 0 0 1 8 0v2 M5 7a2 2 0 0 1 2-2h10a2 2 0 0 1 2 2v14H5V7z M8 14h8v5H8z M7 9h10 M3 11v7 M21 11v7',
 mouse:'M6 9a6 6 0 0 1 12 0v7a6 6 0 0 1-12 0V9z M12 3v7 M6 10h12 M12 6v2',
 cup:'M4 5h12v13a3 3 0 0 1-3 3H7a3 3 0 0 1-3-3V5z M16 7h3a3 3 0 0 1 0 6h-3 M7 2h6'
};
export function icon(name) { return `<svg class="icon" viewBox="0 0 24 24" aria-hidden="true"><path d="${paths[name] || paths.box}"/></svg>`; }
export function hydrateIcons(root = document) { root.querySelectorAll('[data-icon]').forEach(e => { e.innerHTML = icon(e.dataset.icon); }); }
