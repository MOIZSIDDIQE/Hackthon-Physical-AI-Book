import React, { useState, useRef } from 'react';
import { motion, useMotionValue, useTransform, animate } from 'framer-motion';
import BrowserOnly from '@docusaurus/BrowserOnly';

/**
 * Magnetic Button Component
 * Button that slightly follows cursor movement for magnetic effect
 */
function MagneticButtonContent({ children, className = '', href, onClick, ...props }) {
  const buttonRef = useRef(null);
  const [isHovered, setIsHovered] = useState(false);

  const x = useMotionValue(0);
  const y = useMotionValue(0);

  const rotateX = useTransform(y, [-50, 50], [-2, 2]);
  const rotateY = useTransform(x, [-50, 50], [2, -2]);

  const handleMouseMove = (event) => {
    if (!buttonRef.current) return;

    const rect = buttonRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    x.set(event.clientX - centerX);
    y.set(event.clientY - centerY);
  };

  const handleMouseLeave = () => {
    setIsHovered(false);
    // Reset to center with spring animation
    animate(x, 0, { duration: 0.3, ease: "easeOut" });
    animate(y, 0, { duration: 0.3, ease: "easeOut" });
  };

  const buttonClasses = `magnetic-button ${className}`;

  return (
    <motion.a
      ref={buttonRef}
      className={buttonClasses}
      style={{
        rotateX,
        rotateY,
        x,
        y,
        display: 'inline-block',
      }}
      href={href}
      onClick={onClick}
      onMouseMove={handleMouseMove}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={handleMouseLeave}
      whileHover={{
        scale: 1.05,
        boxShadow: "0 0 40px rgba(6, 182, 212, 0.6)"
      }}
      whileTap={{ scale: 0.95 }}
      transition={{
        type: "spring",
        stiffness: 300,
        damping: 25,
        mass: 0.1
      }}
      {...props}
    >
      {children}
    </motion.a>
  );
}

export default function MagneticButton(props) {
  return (
    <BrowserOnly>
      {() => <MagneticButtonContent {...props} />}
    </BrowserOnly>
  );
}