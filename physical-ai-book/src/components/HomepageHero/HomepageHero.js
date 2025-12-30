import React, { useEffect, useRef } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import { FaBrain, FaMicrochip, FaRocket, FaRobot, FaEye, FaHeartbeat } from 'react-icons/fa';

// Icons for the cards
const Icons = {
  Brain: () => <FaBrain size={40} color="currentColor" />,
  Cpu: () => <FaMicrochip size={40} color="currentColor" />,
  Rocket: () => <FaRocket size={40} color="currentColor" />,
  Robot: () => <FaRobot size={40} color="currentColor" />,
  Eye: () => <FaEye size={40} color="currentColor" />,
  Activity: () => <FaHeartbeat size={40} color="currentColor" />
};

function AnimatedHeroContent() {
  const heroRef = useRef(null);
  const gridRef = useRef(null);
  const titleRef = useRef(null);
  const subtitleRef = useRef(null);
  const buttonsRef = useRef(null);

  useEffect(() => {
    const loadGSAP = async () => {
      // Dynamically import GSAP libraries
      const gsapModule = await import('gsap');
      const { ScrollTrigger } = await import('gsap/ScrollTrigger');
      const { useGSAP: useGSAPModule } = await import('@gsap/react');

      const { default: gsap } = gsapModule;

      gsap.registerPlugin(ScrollTrigger);

      // Mouse tracking for 3D grid effect
      const handleMouseMove = (e) => {
        const grid = gridRef.current;
        if (!grid) return;

        const rect = grid.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;

        const centerX = rect.width / 2;
        const centerY = rect.height / 2;

        const rotateX = (y - centerY) / 20;
        const rotateY = (centerX - x) / 20;

        gsap.to(grid, {
          rotationX: rotateX,
          rotationY: rotateY,
          duration: 0.5,
          ease: 'power2.out',
          transformPerspective: 1000,
        });
      };

      // Add mouse move listener
      window.addEventListener('mousemove', handleMouseMove);

      // Scroll-triggered animations
      gsap.to(titleRef.current, {
        y: 0,
        opacity: 1,
        duration: 1.2,
        ease: 'power3.out',
        scrollTrigger: {
          trigger: titleRef.current,
          start: 'top 80%',
          end: 'bottom 20%',
          toggleActions: 'play none none reverse'
        }
      });

      gsap.to(subtitleRef.current, {
        y: 0,
        opacity: 1,
        duration: 1,
        delay: 0.2,
        ease: 'power2.out',
        scrollTrigger: {
          trigger: subtitleRef.current,
          start: 'top 80%',
          end: 'bottom 20%',
          toggleActions: 'play none none reverse'
        }
      });

      gsap.to(buttonsRef.current, {
        y: 0,
        opacity: 1,
        duration: 0.8,
        delay: 0.4,
        ease: 'power2.out',
        scrollTrigger: {
          trigger: buttonsRef.current,
          start: 'top 80%',
          end: 'bottom 20%',
          toggleActions: 'play none none reverse'
        }
      });

      // Grid pulse animation
      gsap.to(gridRef.current, {
        scale: 1.05,
        duration: 3,
        repeat: -1,
        yoyo: true,
        ease: 'sine.inOut',
        scrollTrigger: {
          trigger: heroRef.current,
          start: 'top bottom',
          end: 'bottom top',
          scrub: true
        }
      });

      return () => {
        window.removeEventListener('mousemove', handleMouseMove);
        if (ScrollTrigger) {
          ScrollTrigger.getAll().forEach(trigger => trigger.kill());
        }
      };
    };

    loadGSAP();
  }, []);

  const {siteConfig} = useDocusaurusContext();

  return (
    <header ref={heroRef} className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles['hero-content']}>
          {/* Animated Grid Background */}
          <div ref={gridRef} className={styles['hero-grid']} style={{
            position: 'absolute',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            pointerEvents: 'none',
            transformStyle: 'preserve-3d',
            zIndex: -1
          }}></div>

          <div style={{ position: 'relative', zIndex: 1 }}>
            <h1 ref={titleRef} className={styles.hero__title} style={{ transform: 'translateY(50px)', opacity: 0 }}>
              {siteConfig.title}
            </h1>
            <p ref={subtitleRef} className={styles.hero__subtitle} style={{ transform: 'translateY(30px)', opacity: 0 }}>
              {siteConfig.tagline}
            </p>
            <div ref={buttonsRef} className={styles.buttons} style={{ transform: 'translateY(30px)', opacity: 0 }}>
              <Link
                className={styles['hero-button']}
                to="/docs/part-01-foundations/chapter-01/1.1-concepts">
                Start Learning
              </Link>
              <Link
                className={styles['hero-button-secondary']}
                to="/docs/intro">
                View Curriculum
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function HomepageHeader() {
  return (
    <BrowserOnly>
      {() => <AnimatedHeroContent />}
    </BrowserOnly>
  );
}

export default HomepageHeader;