import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

const HomepageHeader = () => {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <h1 className="hero__title">{siteConfig.title}</h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <p>Learn about digital twin simulation, humanoid robotics, and the robotic nervous system with ROS 2.</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Get Started
              </Link>
              <Link
                className="button button--primary button--lg"
                to="/docs/module-1/intro">
                View Modules
              </Link>
            </div>
          </div>
          <div className="col col--6">
            <div className={styles.heroImageContainer}>
              <img
                src="/img/robo-logo.jpg"
                alt="Humanoid Robotics Logo"
                className={styles.heroImage}
                style={{ width: '100%', maxWidth: '300px', height: 'auto' }}
                onError={(e) => {
                  console.error('Image failed to load:', e);
                  e.currentTarget.style.display = 'none';
                  e.currentTarget.insertAdjacentHTML('afterend', '<div>specify</div>');
                }}
              />
            </div>
          </div>
        </div>
      </div>
    </header>
  );
};

const HomepageContent = () => {
  return (
    <main>
      <div className="container padding-top--md">
        <div className="row">
          <div className="col col--4">
            <div className={styles.featureCard}>
              <h3>Digital Twin Simulation</h3>
              <p>Physics-accurate simulation using Gazebo with Unity visualization for high-fidelity rendering and human-robot interaction.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.featureCard}>
              <h3>Robotic Nervous System</h3>
              <p>Learn ROS 2 fundamentals, nodes, topics, and services to connect your AI agent to the robotic system.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.featureCard}>
              <h3>Sensor Integration</h3>
              <p>Simulate realistic sensors including LiDAR, Depth Cameras, and IMUs for comprehensive perception in robotics.</p>
            </div>
          </div>
        </div>
      </div>
    </main>
  );
};

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Understanding the Robotic Nervous System with ROS 2">
      <HomepageHeader />
      <main>
        <HomepageContent />
      </main>
    </Layout>
  );
}