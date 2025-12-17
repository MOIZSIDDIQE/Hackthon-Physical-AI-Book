import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="hero-content">
          <div className="hero-text">
            <h1 className="hero__title">
              {siteConfig.title}
            </h1>
            <p className="hero__subtitle">
              {siteConfig.tagline}
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg hero-button"
                to="/docs/part-01-foundations/chapter-01/1.1-concepts">
                Start Learning
              </Link>
              <Link
                className="button button--secondary button--lg hero-button-secondary"
                to="/docs/part-01-foundations/chapter-01/1.1-concepts">
                View Curriculum
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function LearningPath() {
  return (
    <section className={styles.learningPath}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className="section-title">Learning Journey</Heading>
            <p className="section-subtitle">Progress from foundational concepts to advanced humanoid systems</p>
          </div>
        </div>
        <div className="row learning-path-cards">
          <div className="col col--4">
            <div className="learning-card">
              <div className="card-icon"> foundations </div>
              <h3>Foundations</h3>
              <p>Master the core principles of Physical AI and humanoid robotics</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="learning-card">
              <div className="card-icon"> systems </div>
              <h3>Systems</h3>
              <p>Understand perception, planning, control, and simulation systems</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="learning-card">
              <div className="card-icon"> applications </div>
              <h3>Applications</h3>
              <p>Apply knowledge to real-world robotics challenges</p>
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
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className="section-title">What You Will Build</Heading>
            <p className="section-subtitle">From simple concepts to advanced humanoid systems</p>
          </div>
        </div>
        <div className="row">
          <div className="col col--4">
            <div className="feature-card">
              <h3>Basic Humanoid Controller</h3>
              <p>Design and implement fundamental control systems for humanoid robots</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="feature-card">
              <h3>Perception Systems</h3>
              <p>Build AI systems that perceive and interact with their environment</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="feature-card">
              <h3>Advanced Behaviors</h3>
              <p>Develop complex locomotion and manipulation capabilities</p>
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
      title={`Physical AI & Humanoid Robotics`}
      description="A comprehensive guide to building intelligent humanoid robots">
      <HomepageHeader />
      <main>
        <LearningPath />
        <WhatYouWillBuild />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
