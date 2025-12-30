import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import HomepageHero from '@site/src/components/HomepageHero';
import Heading from '@theme/Heading';
import styles from './index.module.css';
import { FaBrain, FaMicrochip, FaRocket, FaRobot, FaEye, FaHeartbeat } from 'react-icons/fa';
import { FaBook, FaGraduationCap, FaWrench } from 'react-icons/fa';

// Icons for the sections
const Icons = {
  Brain: () => <FaBrain size={40} color="currentColor" />,
  Cpu: () => <FaMicrochip size={40} color="currentColor" />,
  Rocket: () => <FaRocket size={40} color="currentColor" />,
  Robot: () => <FaRobot size={40} color="currentColor" />,
  Eye: () => <FaEye size={40} color="currentColor" />,
  Activity: () => <FaHeartbeat size={40} color="currentColor" />
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