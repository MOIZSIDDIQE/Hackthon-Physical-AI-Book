import React, { useState, useEffect } from 'react';

/**
 * BrowserOnly Component
 * Ensures children only render in browser environment to prevent SSR mismatches
 */
export default function BrowserOnly({ children, fallback = null }) {
  const [isBrowser, setIsBrowser] = useState(false);

  useEffect(() => {
    setIsBrowser(true);
  }, []);

  if (!isBrowser) {
    return fallback;
  }

  return children;
}