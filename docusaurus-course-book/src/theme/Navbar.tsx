import React, { JSX } from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useColorMode } from '@docusaurus/theme-common';
import styles from './navbar.module.css';

export default function NavbarCustom(): JSX.Element {
  const base = useBaseUrl('/');
  const { colorMode, setColorMode } = useColorMode();

  return (
    <header className={styles.header}>
      <div className={styles.left}>
        <a className={styles.iconLink} href={base}>
          <img src={useBaseUrl('img/logo.png')} alt="Book" className={styles.logo} />
        </a>

        <nav className={styles.navItems}>
          <Link to={useBaseUrl('/auth')} className="button button--secondary">
            Login
          </Link>

          <div style={{marginLeft:12}}>
            <button onClick={() => setColorMode(colorMode === 'dark' ? 'light' : 'dark')}>
              {colorMode === 'dark' ? '☀️' : '🌙'}
            </button>
          </div>

          <a
            className="button button--secondary"
            href="https://github.com/Darakhshan-Imran/TEXT-BOOK"
            target="_blank"
            rel="noreferrer"
            style={{marginLeft:12}}
          >
            GitHub
          </a>
        </nav>
      </div>

      <div className={styles.right}>
        <Link to={useBaseUrl('/docs')} className={styles.titleLink}>
          <span className={styles.title}>Physical AI &amp; Humanoid Robotics</span>
        </Link>
      </div>
    </header>
  );
}