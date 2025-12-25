import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Embodied Intelligence',
  favicon: 'img/favicon.ico',

  url: 'https://Darakhshan-Imran.github.io',
  baseUrl: '/TEXT-BOOK/',

  organizationName: 'Darakhshan-Imran',
  projectName: 'TEXT-BOOK',

  onBrokenLinks: 'warn',
  // onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur', 'zh', 'fr'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
      zh: {
        label: '中文',
        direction: 'ltr',
        htmlLang: 'zh-CN',
      },
      fr: {
        label: 'Français',
        direction: 'ltr',
        htmlLang: 'fr-FR',
      },
    },
  },


  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: 'docs',
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/Darakhshan-Imran/TEXT-BOOK/tree/main/',
        },
        // blog: false, // Disable blog
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/robo.png',
        style: { height: '55px' },
      },
      items: [
        {
          type: 'custom-textbookNavbarItem',
          position: 'left',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          type: 'custom-authNavbarItem',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course by Darakhshan-Imran. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml'],
    },
  } satisfies Preset.ThemeConfig,

  // Enable Mermaid for diagrams
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],
};

export default config;

