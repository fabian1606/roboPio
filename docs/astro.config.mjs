import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

export default defineConfig({
  integrations: [
    starlight({
      title: 'roboPio Dokumentation',
      defaultLocale: 'de',
      locales: {
        root: {
          label: 'Deutsch',
          lang: 'de',
        },
      },
      sidebar: [
        {
          label: 'Übersicht',
          link: '/',
        },
        {
          label: 'Installation',
          link: '/installation/',
        },
        {
          label: 'Ansteuerungsmodi',
          items: [
            { label: 'Direkt-Modus', link: '/modes/direkt/' },
            { label: 'Zylindrisch-Modus', link: '/modes/zylindrisch/' },
            { label: 'Kartesisch-Modus', link: '/modes/kartesisch/' },
          ],
        },
      ],
    }),
  ],
});
