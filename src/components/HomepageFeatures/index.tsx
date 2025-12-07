import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Module 1: ROS 2 Foundations',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn the fundamentals of ROS 2, including environment setup, nodes, topics, and creating basic communication for robot control.
      </>
    ),
  },
  {
    title: 'Module 2: The Digital Twin',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Explore the 'Digital Twin' concept, mastering terminology and simulating physics, sensors, and environments in Gazebo and Unity for realistic robot modeling.
      </>
    ),
  },
  {
    title: 'Module 3: NVIDIA Isaac Sim',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Dive into NVIDIA Isaac Sim, understanding its architecture, setting up simulations, and implementing advanced perception, navigation, and reinforcement learning for humanoid control.
      </>
    ),
  },
  {
    title: 'Module 4: VLA Robotics LLM',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default, // Re-using for the 4th module
    description: (
      <>
        Discover Vision-Language-Action (VLA) systems, enabling humanoid robots to interpret voice commands via Whisper, plan with LLMs, and execute complex actions using ROS 2, perception, and manipulation.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
