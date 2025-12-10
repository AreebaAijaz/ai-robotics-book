import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

// Hero Section Component
function HeroSection() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroBackground}></div>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroLeft}>
            <div className={styles.bookCover}>
              <div className={styles.bookInner}>
                <div className={styles.robotIcon}>🤖</div>
                <div className={styles.bookTitle}>Physical AI</div>
                <div className={styles.bookSubtitle}>& Humanoid Robotics</div>
                <div className={styles.bookEdition}>Complete Guide</div>
              </div>
            </div>
          </div>
          <div className={styles.heroRight}>
            <div className={styles.badges}>
              <span className={styles.badge}>🌟 Open Source</span>
              <span className={styles.badge}>🛠️ Hands-On Learning</span>
              <span className={styles.badge}>🚀 Industry-Ready</span>
            </div>
            <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
            <p className={styles.heroSubtitle}>
              Master the convergence of AI and robotics. Learn ROS 2, simulation,
              NVIDIA Isaac, and Vision-Language-Action models to build intelligent
              humanoid robots that perceive, reason, and act in the physical world.
            </p>
            <div className={styles.buttons}>
              <Link
                className={clsx('button button--lg', styles.primaryBtn)}
                to="/docs/intro">
                Start Learning →
              </Link>
              <Link
                className={clsx('button button--lg', styles.secondaryBtn)}
                href="https://github.com/AreebaAijaz/ai-robotics-book">
                View on GitHub
              </Link>
            </div>
            <div className={styles.stats}>
              <div className={styles.statItem}>
                <span className={styles.statNumber}>4</span>
                <span className={styles.statLabel}>Modules</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statNumber}>15+</span>
                <span className={styles.statLabel}>Chapters</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statNumber}>50+</span>
                <span className={styles.statLabel}>Code Examples</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

// Core Concepts Section
const concepts = [
  {
    icon: '🛠️',
    title: 'ROS 2 Fundamentals',
    description: 'Robotics Middleware & Control',
    points: [
      'Nodes, Topics & Services',
      'Python with rclpy',
      'URDF Robot Modeling',
    ],
    color: '#6366f1',
  },
  {
    icon: '🎮',
    title: 'Simulation & Digital Twins',
    description: 'Virtual Testing Environments',
    points: [
      'Gazebo Physics Simulation',
      'Unity ML Integration',
      'Sensor Simulation',
    ],
    color: '#8b5cf6',
  },
  {
    icon: '🚀',
    title: 'NVIDIA Isaac Platform',
    description: 'AI-Powered Perception',
    points: [
      'Isaac Sim & Isaac ROS',
      'Visual SLAM Navigation',
      'Bipedal Locomotion',
    ],
    color: '#06b6d4',
  },
  {
    icon: '🧠',
    title: 'Vision-Language-Action',
    description: 'Conversational Robotics',
    points: [
      'Voice-to-Action Pipelines',
      'LLM Cognitive Planning',
      'End-to-End VLA Models',
    ],
    color: '#10b981',
  },
];

function CoreConceptsSection() {
  return (
    <section className={styles.conceptsSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>The Physical AI Journey</h2>
          <p className={styles.sectionSubtitle}>
            Four comprehensive modules taking you from robotics fundamentals to cutting-edge AI integration
          </p>
        </div>
        <div className={styles.conceptsGrid}>
          {concepts.map((concept, idx) => (
            <div
              key={idx}
              className={styles.conceptCard}
              style={{'--card-color': concept.color}}
            >
              <div className={styles.conceptIcon}>{concept.icon}</div>
              <h3 className={styles.conceptTitle}>{concept.title}</h3>
              <p className={styles.conceptDesc}>{concept.description}</p>
              <ul className={styles.conceptPoints}>
                {concept.points.map((point, i) => (
                  <li key={i}>{point}</li>
                ))}
              </ul>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Learning Path Section
const pathSteps = [
  {
    number: '01',
    title: 'Learn ROS 2',
    description: 'Build robotic systems with industry-standard middleware',
    icon: '📚',
  },
  {
    number: '02',
    title: 'Master Simulation',
    description: 'Create digital twins for safe robot development',
    icon: '🎯',
  },
  {
    number: '03',
    title: 'Deploy with Isaac',
    description: 'Leverage NVIDIA AI for perception & navigation',
    icon: '⚡',
  },
  {
    number: '04',
    title: 'Build Capstone',
    description: 'Create an autonomous humanoid robot system',
    icon: '🏆',
  },
];

function LearningPathSection() {
  return (
    <section className={styles.pathSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Your Robotics Mastery Path</h2>
          <p className={styles.sectionSubtitle}>
            A structured learning journey from fundamentals to building complete humanoid robot systems
          </p>
        </div>
        <div className={styles.pathContainer}>
          <div className={styles.pathLine}></div>
          {pathSteps.map((step, idx) => (
            <div key={idx} className={styles.pathStep}>
              <div className={styles.pathNumber}>{step.number}</div>
              <div className={styles.pathContent}>
                <span className={styles.pathIcon}>{step.icon}</span>
                <h3 className={styles.pathTitle}>{step.title}</h3>
                <p className={styles.pathDesc}>{step.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Why This Matters Section
function WhyThisMattersSection() {
  return (
    <section className={styles.comparisonSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>The Paradigm Shift</h2>
          <p className={styles.sectionSubtitle}>
            Why Physical AI represents the next frontier in robotics
          </p>
        </div>
        <div className={styles.comparisonGrid}>
          <div className={clsx(styles.comparisonCard, styles.traditional)}>
            <div className={styles.comparisonHeader}>
              <span className={styles.comparisonIcon}>⚙️</span>
              <h3>Traditional Robotics</h3>
            </div>
            <ul className={styles.comparisonList}>
              <li>
                <span className={styles.crossIcon}>✗</span>
                Manual programming for each task
              </li>
              <li>
                <span className={styles.crossIcon}>✗</span>
                Limited adaptability to new situations
              </li>
              <li>
                <span className={styles.crossIcon}>✗</span>
                Rigid, pre-defined behaviors
              </li>
              <li>
                <span className={styles.crossIcon}>✗</span>
                Complex integration challenges
              </li>
              <li>
                <span className={styles.crossIcon}>✗</span>
                Requires robotics expertise only
              </li>
            </ul>
          </div>
          <div className={styles.vsIndicator}>
            <span>VS</span>
          </div>
          <div className={clsx(styles.comparisonCard, styles.physicalAI)}>
            <div className={styles.comparisonHeader}>
              <span className={styles.comparisonIcon}>🤖</span>
              <h3>Physical AI Way</h3>
            </div>
            <ul className={styles.comparisonList}>
              <li>
                <span className={styles.checkIcon}>✓</span>
                Learning from demonstration & language
              </li>
              <li>
                <span className={styles.checkIcon}>✓</span>
                Dynamic adaptation to environments
              </li>
              <li>
                <span className={styles.checkIcon}>✓</span>
                Intelligent decision-making
              </li>
              <li>
                <span className={styles.checkIcon}>✓</span>
                Unified perception-action pipeline
              </li>
              <li>
                <span className={styles.checkIcon}>✓</span>
                Bridges AI and robotics expertise
              </li>
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}

// Call to Action Section
function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <div className={styles.ctaContent}>
          <h2>Ready to Build Intelligent Robots?</h2>
          <p>
            Join the Physical AI revolution. Start your journey from AI practitioner
            to robotics engineer today.
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx('button button--lg', styles.ctaPrimaryBtn)}
              to="/docs/intro">
              Begin Your Journey
            </Link>
            <Link
              className={clsx('button button--lg', styles.ctaSecondaryBtn)}
              to="/docs/module-1-ros2/architecture">
              Jump to Module 1
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

// Main Homepage Component
export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} - Complete Educational Resource`}
      description="Master Physical AI and Humanoid Robotics: ROS 2, Simulation, NVIDIA Isaac, and Vision-Language-Action models">
      <HeroSection />
      <main>
        <CoreConceptsSection />
        <LearningPathSection />
        <WhyThisMattersSection />
        <CTASection />
      </main>
    </Layout>
  );
}
