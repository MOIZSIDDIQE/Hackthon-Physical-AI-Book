import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import HomepageHero from '@site/src/components/HomepageHero';
import Heading from '@theme/Heading';
import styles from './index.module.css';

// --- VIP ICONS (Inko waise hi rakha hai kyunki ye styling ke liye zaroori hain) ---
const Icons = {
  Brain: () => (
    <svg xmlns="http://www.w3.org/2000/svg" width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M9.5 2A2.5 2.5 0 0 1 12 4.5v15a2.5 2.5 0 0 1-4.96.44 2.5 2.5 0 0 1-2.96-3.08 3 3 0 0 1-.34-5.58 2.5 2.5 0 0 1 1.32-4.24 2.5 2.5 0 0 1 1.98-3A2.5 2.5 0 0 1 9.5 2Z"/><path d="M14.5 2A2.5 2.5 0 0 0 12 4.5v15a2.5 2.5 0 0 0 4.96.44 2.5 2.5 0 0 0 2.96-3.08 3 3 0 0 0 .34-5.58 2.5 2.5 0 0 0-1.32-4.24 2.5 2.5 0 0 0-1.98-3A2.5 2.5 0 0 0 14.5 2Z"/></svg>
  ),
  Cpu: () => (
    <svg xmlns="http://www.w3.org/2000/svg" width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/><path d="M9 1V4"/><path d="M15 1V4"/><path d="M9 20V23"/><path d="M15 20V23"/><path d="M20 9H23"/><path d="M20 14H23"/><path d="M1 9H4"/><path d="M1 14H4"/></svg>
  ),
  Rocket: () => (
    <svg xmlns="http://www.w3.org/2000/svg" width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M4.5 16.5c-1.5 1.26-2 5-2 5s3.74-.5 5-2c.71-.84.7-2.13-.09-2.91a2.18 2.18 0 0 0-2.91-.09z"/><path d="m12 15-3-3a22 22 0 0 1 2-3.95A12.88 12.88 0 0 1 22 2c0 2.72-.78 7.5-6 11a22.35 22.35 0 0 1-4 2z"/><path d="M9 12H4s.55-3.03 2-4c1.62-1.08 5 0 5 0"/><path d="M12 15v5s3.03-.55 4-2c1.08-1.62 0-5 0-5"/></svg>
  ),
  Robot: () => (
    <svg xmlns="http://www.w3.org/2000/svg" width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><rect width="18" height="10" x="3" y="11" rx="2"/><circle cx="12" cy="5" r="2"/><path d="M12 7v4"/><line x1="8" x2="8" y1="16" y2="16"/><line x1="16" x2="16" y1="16" y2="16"/></svg>
  ),
  Eye: () => (
    <svg xmlns="http://www.w3.org/2000/svg" width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M2 12s3-7 10-7 10 7 10 7-3 7-10 7-10-7-10-7Z"/><circle cx="12" cy="12" r="3"/></svg>
  ),
  Activity: () => (
    <svg xmlns="http://www.w3.org/2000/svg" width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg>
  )
};

function LearningPath() {
  return (
    <section className={styles.learningPath}>
      <div className="container">
        {/* Title Row */}
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles['section-title']}>Learning Journey</Heading>
            <p className={styles['section-subtitle']}>From Core Concepts to Advanced Robotics</p>
          </div>
        </div>
        
        {/* Cards Row - Simplified Layout Logic */}
        <div className="row">
          
          {/* Card 1 */}
          <div className={clsx('col col--4')}>
            <div className={styles['learning-card']}>
              <div className={styles['card-icon']}>
                <Icons.Brain />
              </div>
              <h3>Foundations</h3>
              <p>Master the core principles of Physical AI, mathematics, and kinematics.</p>
            </div>
          </div>

          {/* Card 2 */}
          <div className={clsx('col col--4')}>
            <div className={styles['learning-card']}>
              <div className={styles['card-icon']}>
                <Icons.Cpu />
              </div>
              <h3>Systems</h3>
              <p>Deep dive into perception, planning, control theory, and simulation.</p>
            </div>
          </div>

          {/* Card 3 */}
          <div className={clsx('col col--4')}>
            <div className={styles['learning-card']}>
              <div className={styles['card-icon']}>
                <Icons.Rocket />
              </div>
              <h3>Applications</h3>
              <p>Deploy real-world solutions and build autonomous humanoid behaviors.</p>
            </div>
          </div>

        </div>
      </div>
    </section>
  );
}

function WhatYouWillBuild() {
  return (
    <section className={styles.whatYouWillBuild}>
      <div className="container">
        {/* Title Row */}
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles['section-title']}>What You Will Build</Heading>
            <p className={styles['section-subtitle']}>Practical Projects & Implementations</p>
          </div>
        </div>

        {/* Features Row */}
        <div className="row">
          
          {/* Feature 1 */}
          <div className={clsx('col col--4')}>
            <div className={styles['feature-card']}>
              <div className={styles['card-icon']}>
                 <Icons.Robot />
              </div>
              <h3>Humanoid Controller</h3>
              <p>Design stable walking and balancing algorithms for bipedal robots.</p>
            </div>
          </div>

          {/* Feature 2 */}
          <div className={clsx('col col--4')}>
            <div className={styles['feature-card']}>
              <div className={styles['card-icon']}>
                 <Icons.Eye />
              </div>
              <h3>Perception Stack</h3>
              <p>Build computer vision systems to detect objects and map environments.</p>
            </div>
          </div>

          {/* Feature 3 */}
          <div className={clsx('col col--4')}>
            <div className={styles['feature-card']}>
              <div className={styles['card-icon']}>
                 <Icons.Activity />
              </div>
              <h3>Complex Behaviors</h3>
              <p>Develop advanced manipulation and interaction capabilities.</p>
            </div>
          </div>

        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="A comprehensive guide to building intelligent humanoid robots">
      <HomepageHero />
      <main>
        <LearningPath />
        <WhatYouWillBuild />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}