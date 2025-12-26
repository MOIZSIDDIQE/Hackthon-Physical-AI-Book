import React, { useEffect, useRef } from 'react';
import { motion, useInView, useAnimation } from 'framer-motion';
import BrowserOnly from '@docusaurus/BrowserOnly';

/**
 * FadeInView Component
 * Reusable component for scroll-triggered fade-in animations
 */
function FadeInViewContent({ children, delay = 0, duration = 0.6, stagger = false, ...props }) {
  const ref = useRef(null);
  const isInView = useInView(ref, { once: true, margin: "-100px" });
  const mainControls = useAnimation();
  const slideControls = useAnimation();

  useEffect(() => {
    if (isInView) {
      mainControls.start("visible", { delay });
      if (stagger) {
        slideControls.start("visible", { delay: delay + 0.1 });
      }
    }
  }, [isInView, delay, mainControls, slideControls, stagger]);

  const container = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        delayChildren: 0.1,
        staggerChildren: 0.1
      }
    }
  };

  const item = {
    hidden: { y: 20, opacity: 0 },
    visible: {
      y: 0,
      opacity: 1,
      transition: {
        duration: duration,
        ease: "easeOut"
      }
    }
  };

  const animationProps = stagger
    ? { variants: container, initial: "hidden", animate: mainControls }
    : { initial: "hidden", animate: mainControls, variants: { hidden: { opacity: 0 }, visible: { opacity: 1, transition: { duration, delay } } } };

  return (
    <motion.div
      ref={ref}
      {...animationProps}
      {...(stagger && { variants: container })}
    >
      {stagger ? (
        <motion.div variants={item}>
          {children}
        </motion.div>
      ) : (
        children
      )}
    </motion.div>
  );
}

export default function FadeInView(props) {
  return (
    <BrowserOnly>
      {() => <FadeInViewContent {...props} />}
    </BrowserOnly>
  );
}