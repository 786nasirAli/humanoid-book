// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Humanoid AI Book',
  tagline: 'Understanding the Robotic Nervous System with ROS 2',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://786nasirali.github.io', // Updated with your actual GitHub Pages URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages deployment, it's often '/<REPO_NAME>/'
  baseUrl: '/humanoid-book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: '786nasirAli', // Usually your GitHub org/user name.
  projectName: 'humanoid-book', // Usually your repo name.
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/786nasirAli/humanoid-book/edit/main/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/786nasirAli/humanoid-book/edit/main/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themes: ['@docusaurus/theme-live-codeblock'],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg', // Placeholder - replace with actual image
      navbar: {
        title: '',
        hideOnScroll: false,
        style: 'dark',
        logo: {
          alt: 'Humanoid AI Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            position: 'left',
            to: '/',
            label: 'Home',
          },
          {
            href: 'https://github.com/786nasirAli/humanoid-book',
            position: 'right',
            className: 'navbar-external-link',
            'aria-label': 'GitHub repository',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Module 1: The Robotic Nervous System',
                to: '/docs/module-1/intro',
              },
              {
                label: 'Module 2: The Digital Twin (Gazebo & Unity)',
                to: '/docs/module-2/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros2',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/ros',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/786nasirAli/humanoid-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;