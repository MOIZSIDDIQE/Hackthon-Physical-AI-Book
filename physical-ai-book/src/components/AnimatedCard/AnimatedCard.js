import React, { useRef, useState } from 'react';
import { 
  motion, 
  useMotionValue, 
  useTransform, 
  useSpring, 
  useMotionTemplate 
} from 'framer-motion';

/**
 * 💎 ULTIMATE VIP CARD (Self-Contained Style)
 * No external CSS file needed.
 */
export default function AnimatedCard({
  children,
  icon,
  title,
  description,
  variant = 'default' // 'learning', 'feature', 'default'
}) {
  const ref = useRef(null);
  const [isHovered, setIsHovered] = useState(false);

  // --- PHYSICS ENGINE ---
  const x = useMotionValue(0);
  const y = useMotionValue(0);

  // Smooth Springs (Mass/Stiffness controls the "weight" feel)
  const mouseX = useSpring(x, { stiffness: 500, damping: 30 });
  const mouseY = useSpring(y, { stiffness: 500, damping: 30 });

  // 3D Rotation Mapping
  const rotateX = useTransform(mouseY, [-0.5, 0.5], ["7deg", "-7deg"]);
  const rotateY = useTransform(mouseX, [-0.5, 0.5], ["-7deg", "7deg"]);

  // Dynamic Spotlight Logic
  const spotlightX = useTransform(mouseX, [-0.5, 0.5], ["0%", "100%"]);
  const spotlightY = useTransform(mouseY, [-0.5, 0.5], ["0%", "100%"]);
  const spotlightBackground = useMotionTemplate`radial-gradient(
    600px circle at ${spotlightX} ${spotlightY}, 
    rgba(255, 255, 255, 0.1), 
    transparent 80%
  )`;

  // --- COLOR THEMES (Based on Variant) ---
  const getThemeColor = () => {
    switch(variant) {
      case 'learning': return '#3b82f6'; // Blue
      case 'feature': return '#a855f7';  // Purple
      default: return '#06b6d4';         // Cyan
    }
  };
  const themeColor = getThemeColor();

  // --- EVENT HANDLERS ---
  const handleMouseMove = (e) => {
    if (!ref.current) return;
    const rect = ref.current.getBoundingClientRect();
    const width = rect.width;
    const height = rect.height;
    const mouseXPos = e.clientX - rect.left;
    const mouseYPos = e.clientY - rect.top;
    const xPct = (mouseXPos / width) - 0.5;
    const yPct = (mouseYPos / height) - 0.5;
    x.set(xPct);
    y.set(yPct);
  };

  const handleMouseLeave = () => {
    setIsHovered(false);
    x.set(0);
    y.set(0);
  };

  // --- INLINE STYLES (No External CSS Needed) ---
  const styles = {
    container: {
      perspective: '1000px', // Critical for 3D effect
      width: '100%',
      height: '100%',
      cursor: 'pointer',
    },
    card: {
      position: 'relative',
      width: '100%',
      height: '100%',
      borderRadius: '20px',
      background: 'rgba(17, 24, 39, 0.7)', // Dark semi-transparent
      backdropFilter: 'blur(20px)',
      WebkitBackdropFilter: 'blur(20px)',
      border: `1px solid rgba(255, 255, 255, 0.1)`,
      overflow: 'hidden',
      transformStyle: 'preserve-3d', // Critical for Parallax
      boxShadow: isHovered 
        ? `0 20px 50px -10px rgba(0,0,0,0.5), 0 0 30px -10px ${themeColor}80` 
        : '0 10px 30px -10px rgba(0,0,0,0.5)',
      transition: 'box-shadow 0.3s ease, border-color 0.3s ease',
      borderColor: isHovered ? `${themeColor}80` : 'rgba(255, 255, 255, 0.1)',
    },
    spotlight: {
      position: 'absolute',
      inset: 0,
      opacity: isHovered ? 1 : 0,
      transition: 'opacity 0.5s ease',
      pointerEvents: 'none',
      zIndex: 1,
    },
    contentContainer: {
      position: 'relative',
      padding: '2rem',
      height: '100%',
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'flex-start',
      zIndex: 10,
      transform: 'translateZ(20px)', // Pushes content forward
    },
    iconWrapper: {
      marginBottom: '1.5rem',
      padding: '12px',
      borderRadius: '12px',
      background: 'rgba(255, 255, 255, 0.05)',
      border: '1px solid rgba(255, 255, 255, 0.1)',
      color: 'white',
      transform: 'translateZ(40px)', // Icon floats higher
      boxShadow: '0 4px 15px rgba(0,0,0,0.2)',
    },
    title: {
      fontSize: '1.5rem',
      fontWeight: 'bold',
      marginBottom: '0.75rem',
      color: 'white',
      transform: 'translateZ(30px)',
      transition: 'color 0.3s ease',
      background: isHovered ? `linear-gradient(90deg, #fff, ${themeColor})` : 'none',
      WebkitBackgroundClip: isHovered ? 'text' : 'none',
      WebkitTextFillColor: isHovered ? 'transparent' : 'white',
    },
    description: {
      color: '#94a3b8', // Slate-400
      lineHeight: '1.6',
      fontSize: '1rem',
      transform: 'translateZ(20px)',
      transition: 'color 0.3s ease',
    }
  };

  return (
    <motion.div
      ref={ref}
      style={styles.container}
      onMouseMove={handleMouseMove}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={handleMouseLeave}
      initial={{ opacity: 0, y: 50 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true, margin: "-50px" }}
      transition={{ duration: 0.6, ease: "easeOut" }}
    >
      <motion.div
        style={{
          ...styles.card,
          rotateX,
          rotateY,
        }}
      >
        {/* Spotlight Layer */}
        <motion.div
          style={{
            ...styles.spotlight,
            background: spotlightBackground
          }}
        />

        {/* Content Layer */}
        <div style={styles.contentContainer}>
          {icon && (
            <div style={styles.iconWrapper}>
              {icon}
            </div>
          )}

          {title && (
            <h3 style={styles.title}>
              {title}
            </h3>
          )}

          {description && (
            <p style={styles.description}>
              {description}
            </p>
          )}
          
          {children}
        </div>
      </motion.div>
    </motion.div>
  );
}