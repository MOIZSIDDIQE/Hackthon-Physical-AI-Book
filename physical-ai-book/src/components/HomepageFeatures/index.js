import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Comprehensive Curriculum',
    description: (
      <>
        Complete learning path from Physical AI fundamentals to advanced humanoid systems,
        with hands-on practical exercises and real-world applications.
      </>
    ),
  },
  {
    title: 'MIT-Level Rigor',
    description: (
      <>
        Mathematically rigorous approach with detailed explanations of kinematics,
        dynamics, control theory, and AI integration for robotics.
      </>
    ),
  },
  {
    title: 'Industry-Ready Skills',
    description: (
      <>
        Build practical skills using ROS2, Gazebo, and modern robotics frameworks
        that are directly applicable in research and industry.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <div className={styles.featureIcon}>
          <span className={styles.icon}>🤖</span>
        </div>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className="section-title">Why This Curriculum</Heading>
            <p className="section-subtitle">World-class robotics education designed for the future</p>
          </div>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
