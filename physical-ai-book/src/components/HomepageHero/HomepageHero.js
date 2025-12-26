import React, { useEffect, useRef } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

// Icons for the cards
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