import React, { useEffect } from 'react';
import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import clsx from 'clsx';
import styles from './index.module.css';
import { useAuth } from '@site/src/auth/auth-context';
import useBaseUrl from '@docusaurus/useBaseUrl';

export default function Home(): ReactNode {
  const { user, isLoading } = useAuth();
  const authUrl = useBaseUrl('/auth');
  const docsUrl = useBaseUrl('/docs');

  // Determine where "Get Started" should redirect
  const getStartedUrl = user ? docsUrl : authUrl;
  const getStartedLabel = user ? 'Go to Textbook' : 'Get Started';

  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="A comprehensive guide to embodied intelligence, covering foundations, ROS2, simulation, Isaac, VLA, and deployment">
      <main>
        {/* Hero Section */}
        <section className={styles.hero}>
          <div className={styles.heroContainer}>
            <Heading as="h1" className={styles.heroTitle}>
              Physical AI & Humanoid Robotics
            </Heading>
            <p className={styles.heroSubtitle}>
              Master the complete landscape of embodied intelligence—from foundational concepts to production-ready systems
            </p>
            <div className={styles.heroButtons}>
              <Link
                className={clsx('button button--lg', styles.buttonPrimary)}
                to={getStartedUrl}>
                {getStartedLabel}
              </Link>
              <Link
                className={clsx('button button--lg', styles.buttonSecondary)}
                to="#curriculum">
                Explore Curriculum
              </Link>
            </div>
          </div>
        </section>

        {/* Features Section */}
        <section className={styles.featuresSection}>
          <div className={styles.container}>
            <Heading as="h2" className={styles.sectionTitle}>
              What You'll Learn
            </Heading>
            <div className={styles.featureGrid}>
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🤖</div>
                <Heading as="h3">Embodied Intelligence</Heading>
                <p>
                  Understand the core principles of physical AI, where intelligence emerges from interaction with the physical world. Learn how robots sense, think, and act in real environments.
                </p>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🏗️</div>
                <Heading as="h3">ROS2 & Ecosystems</Heading>
                <p>
                  Master the Robot Operating System 2, learn middleware architecture, and integrate with the broader robotics ecosystem for scalable, production-grade systems.
                </p>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🔬</div>
                <Heading as="h3">Simulation & Modeling</Heading>
                <p>
                  Develop and test complex robotic behaviors in simulation before deployment. Learn physics simulation, computer vision, and digital twins.
                </p>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>⚡</div>
                <Heading as="h3">NVIDIA Isaac Platform</Heading>
                <p>
                  Leverage cutting-edge tools for AI-powered robotics. Explore visual understanding, reinforcement learning, and high-performance simulation.
                </p>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🧠</div>
                <Heading as="h3">Vision Language Models</Heading>
                <p>
                  Integrate foundation models into robotic systems. Learn how VLAs enable robots to understand complex visual scenes and natural language instructions.
                </p>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🚀</div>
                <Heading as="h3">Production Deployment</Heading>
                <p>
                  Move from research to real-world systems. Master deployment strategies, edge computing, safety considerations, and continuous operation.
                </p>
              </div>
            </div>
          </div>
        </section>

        {/* Curriculum Section */}
        <section id="curriculum" className={styles.curriculumSection}>
          <div className={styles.container}>
            <Heading as="h2" className={styles.sectionTitle}>
              Comprehensive Curriculum
            </Heading>
            <div className={styles.curriculumGrid}>
              <div className={styles.part}>
                <Heading as="h3" className={styles.partTitle}>Part 1: Foundations</Heading>
                <ul className={styles.chapters}>
                  <li>Chapter 1: Embodied Intelligence</li>
                  <li>Chapter 2: Robotics Ecosystem</li>
                </ul>
              </div>

              <div className={styles.part}>
                <Heading as="h3" className={styles.partTitle}>Part 2: ROS2</Heading>
                <ul className={styles.chapters}>
                  <li>Architecture & Components</li>
                  <li>Middleware & Communication</li>
                </ul>
              </div>

              <div className={styles.part}>
                <Heading as="h3" className={styles.partTitle}>Part 3: Simulation</Heading>
                <ul className={styles.chapters}>
                  <li>Physics Engines</li>
                  <li>Digital Environments</li>
                </ul>
              </div>

              <div className={styles.part}>
                <Heading as="h3" className={styles.partTitle}>Part 4: Isaac</Heading>
                <ul className={styles.chapters}>
                  <li>Vision & Perception</li>
                  <li>AI Integration</li>
                </ul>
              </div>

              <div className={styles.part}>
                <Heading as="h3" className={styles.partTitle}>Part 5: Vision Language Models</Heading>
                <ul className={styles.chapters}>
                  <li>Foundation Models</li>
                  <li>VLA Integration</li>
                </ul>
              </div>

              <div className={styles.part}>
                <Heading as="h3" className={styles.partTitle}>Part 6: Deployment</Heading>
                <ul className={styles.chapters}>
                  <li>Real-World Systems</li>
                  <li>Production Practices</li>
                </ul>
              </div>
            </div>
            <div className={styles.curriculumCTA}>
              <Link
                className={clsx('button button--lg', styles.buttonPrimary)}
                to={getStartedUrl}>
                {user ? 'Begin with Part 1: Foundations' : 'Login to Start Learning'}
              </Link>
            </div>
          </div>
        </section>

        {/* Who This Is For */}
        <section className={styles.audienceSection}>
          <div className={styles.container}>
            <Heading as="h2" className={styles.sectionTitle}>
              Who Is This For?
            </Heading>
            <div className={styles.audienceGrid}>
              <div className={styles.audienceCard}>
                <p className={styles.audienceTitle}>Roboticists & Engineers</p>
                <p>
                  Build expertise in hardware design, control systems, and real-world deployment of humanoid and mobile robots.
                </p>
              </div>

              <div className={styles.audienceCard}>
                <p className={styles.audienceTitle}>AI & ML Researchers</p>
                <p>
                  Bridge the gap between machine learning research and embodied AI systems. Master multimodal models and reinforcement learning in robotic contexts.
                </p>
              </div>

              <div className={styles.audienceCard}>
                <p className={styles.audienceTitle}>Computer Vision Specialists</p>
                <p>
                  Apply your expertise to robotic perception systems. Learn sensor fusion, 3D vision, and real-time processing pipelines.
                </p>
              </div>

              <div className={styles.audienceCard}>
                <p className={styles.audienceTitle}>Students & Enthusiasts</p>
                <p>
                  Launch your journey in physical AI with structured, hands-on guidance from foundational concepts through advanced topics.
                </p>
              </div>
            </div>
          </div>
        </section>

        {/* Getting Started CTA */}
        <section className={styles.ctaSection}>
          <div className={styles.container}>
            <Heading as="h2" className={styles.sectionTitle}>
              Ready to Master Physical AI?
            </Heading>
            <p className={styles.ctaSubtext}>
              Start with the foundations and progress through real-world applications and deployment strategies.
            </p>
            <Link
              className={clsx('button button--lg', styles.buttonPrimary)}
              to={getStartedUrl}>
              {user ? 'Continue Learning' : 'Get Started Now'}
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}
