import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Embodied Intelligence',
  favicon: 'img/favicon.ico',

  url: 'https://Darakhshan-Imran.github.io',
  baseUrl: '/',

  organizationName: 'Darakhshan-Imran',
  projectName: 'TEXT-BOOK',

  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'ignore',
  onDuplicateRoutes: 'warn',
  markdown: {
    format: 'detect',
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
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
        blog: false, // Disable blog
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          to: '/docs',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/Darakhshan-Imran/TEXT-BOOK',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Course by Darakhshan-Imran. Built with Docusaurus.`,
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

// // This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// const config: Config = {
//   title: 'My Site',
//   tagline: 'Dinosaurs are cool',
//   favicon: 'img/favicon.ico',

//   // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
//   future: {
//     v4: true, // Improve compatibility with the upcoming Docusaurus v4
//   },

//   // Set the production url of your site here
//   url: 'https://your-docusaurus-site.example.com',
//   // Set the /<baseUrl>/ pathname under which your site is served
//   // For GitHub pages deployment, it is often '/<projectName>/'
//   baseUrl: '/',

//   // GitHub pages deployment config.
//   // If you aren't using GitHub pages, you don't need these.
//   organizationName: 'facebook', // Usually your GitHub org/user name.
//   projectName: 'docusaurus', // Usually your repo name.

//   onBrokenLinks: 'throw',

//   // Even if you don't use internationalization, you can use this field to set
//   // useful metadata like html lang. For example, if your site is Chinese, you
//   // may want to replace "en" with "zh-Hans".
//   i18n: {
//     defaultLocale: 'en',
//     locales: ['en'],
//   },

//   presets: [
//     [
//       'classic',
//       {
//         docs: {
//           sidebarPath: './sidebars.ts',
//           // Please change this to your repo.
//           // Remove this to remove the "edit this page" links.
//           editUrl:
//             'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
//         },
//         blog: {
//           showReadingTime: true,
//           feedOptions: {
//             type: ['rss', 'atom'],
//             xslt: true,
//           },
//           // Please change this to your repo.
//           // Remove this to remove the "edit this page" links.
//           editUrl:
//             'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
//           // Useful options to enforce blogging best practices
//           onInlineTags: 'warn',
//           onInlineAuthors: 'warn',
//           onUntruncatedBlogPosts: 'warn',
//         },
//         theme: {
//           customCss: './src/css/custom.css',
//         },
//       } satisfies Preset.Options,
//     ],
//   ],

//   themeConfig: {
//     // Replace with your project's social card
//     image: 'img/docusaurus-social-card.jpg',
//     colorMode: {
//       respectPrefersColorScheme: true,
//     },
//     navbar: {
//       title: 'My Site',
//       logo: {
//         alt: 'My Site Logo',
//         src: 'img/logo.svg',
//       },
//       items: [
//         {
//           type: 'docSidebar',
//           sidebarId: 'tutorialSidebar',
//           position: 'left',
//           label: 'Tutorial',
//         },
//         {to: '/blog', label: 'Blog', position: 'left'},
//         {
//           href: 'https://github.com/facebook/docusaurus',
//           label: 'GitHub',
//           position: 'right',
//         },
//       ],
//     },
//     footer: {
//       style: 'dark',
//       links: [
//         {
//           title: 'Docs',
//           items: [
//             {
//               label: 'Tutorial',
//               to: '/docs/intro',
//             },
//           ],
//         },
//         {
//           title: 'Community',
//           items: [
//             {
//               label: 'Stack Overflow',
//               href: 'https://stackoverflow.com/questions/tagged/docusaurus',
//             },
//             {
//               label: 'Discord',
//               href: 'https://discordapp.com/invite/docusaurus',
//             },
//             {
//               label: 'X',
//               href: 'https://x.com/docusaurus',
//             },
//           ],
//         },
//         {
//           title: 'More',
//           items: [
//             {
//               label: 'Blog',
//               to: '/blog',
//             },
//             {
//               label: 'GitHub',
//               href: 'https://github.com/facebook/docusaurus',
//             },
//           ],
//         },
//       ],
//       copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
//     },
//     prism: {
//       theme: prismThemes.github,
//       darkTheme: prismThemes.dracula,
//     },
//   } satisfies Preset.ThemeConfig,
// };

// export default config;
