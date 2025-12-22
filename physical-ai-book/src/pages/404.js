import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import styles from './404.module.css';

function NotFound() {
  return (
    <Layout title="Page Not Found" description="The requested page could not be found">
      <main className={styles.container}>
        <div className={styles.content}>
          <div className={styles.illustration}>
            <div className={styles.robotIcon}>🤖</div>
          </div>
          <h1 className={styles.title}>404</h1>
          <p className={styles.subtitle}>Page Not Found</p>
          <p className={styles.description}>
            We couldn't find the page you're looking for. It might have been moved,
            deleted, or the URL might be incorrect.
          </p>
          <div className={styles.actions}>
            <Link to="/" className={clsx('button button--primary', styles.primaryButton)}>
              Go to Homepage
            </Link>
            <Link to="/docs/intro" className={clsx('button button--secondary', styles.secondaryButton)}>
              Browse Documentation
            </Link>
          </div>
          <div className={styles.suggestions}>
            <h3>Popular Pages</h3>
            <ul className={styles.suggestionList}>
              <li><Link to="/docs/intro">Introduction to Physical AI</Link></li>
              <li><Link to="/docs/part-01-foundations/chapter-01/1.1-concepts">Robotics Basics</Link></li>
              <li><Link to="/docs/part-05-robot-control/chapter-01/1.1-concepts">Robot Control</Link></li>
              <li><Link to="/docs/part-03-ai-perception/chapter-01/1.1-concepts">AI Perception</Link></li>
            </ul>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default NotFound;