import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import { FaBook, FaGraduationCap, FaWrench } from 'react-icons/fa';

// Icons for the features
const Icons = {
  Book: () => <FaBook size={40} color="currentColor" />,
  Graduation: () => <FaGraduationCap size={40} color="currentColor" />,
  Wrench: () => <FaWrench size={40} color="currentColor" />
};

const FeatureList = [
  {
    title: 'Comprehensive Curriculum',
    description: (
      <>
        Complete learning path from Physical AI fundamentals to advanced humanoid systems,
        with hands-on practical exercises and real-world applications.
      </>
    ),
    icon: <Icons.Book />
  },
  {
    title: 'MIT-Level Rigor',
    description: (
      <>
        Mathematically rigorous approach with detailed explanations of kinematics,
        dynamics, control theory, and AI integration for robotics.
      </>
    ),
    icon: <Icons.Graduation />
  },
  {
    title: 'Industry-Ready Skills',
    description: (
      <>
        Build practical skills using ROS2, Gazebo, and modern robotics frameworks
        that are directly applicable in research and industry.
      </>
    ),
    icon: <Icons.Wrench />
  },
];

function Feature({title, description, icon}) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles['feature-card']}>
        <div className={styles['card-icon']}>
          {icon}
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
            <Heading as="h2" className={styles['section-title']}>Why This Curriculum</Heading>
            <p className={styles['section-subtitle']}>World-class robotics education designed for the future</p>
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
