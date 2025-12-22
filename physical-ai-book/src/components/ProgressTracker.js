import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './ProgressTracker.module.css';

const ProgressTracker = () => {
  const [progress, setProgress] = useState({});
  const [currentChapter, setCurrentChapter] = useState(1);

  // Mock chapter data - in a real app this would come from your documentation structure
  const chapters = [
    { id: 1, title: 'Foundations', completed: false },
    { id: 2, title: 'Humanoid Design', completed: false },
    { id: 3, title: 'AI Perception', completed: false },
    { id: 4, title: 'Motion Planning', completed: false },
    { id: 5, title: 'Robot Control', completed: false },
    { id: 6, title: 'Simulation', completed: false },
    { id: 7, title: 'Kinematics', completed: false },
    { id: 8, title: 'Robot Hardware', completed: false },
    { id: 9, title: 'Ethics', completed: false },
    { id: 10, title: 'Applications', completed: false },
    { id: 11, title: 'Research', completed: false },
    { id: 12, title: 'Future of Humanoids', completed: false },
  ];

  useEffect(() => {
    // Load progress from localStorage
    const savedProgress = localStorage.getItem('bookProgress');
    if (savedProgress) {
      setProgress(JSON.parse(savedProgress));
    }

    // Load current chapter from localStorage
    const savedChapter = localStorage.getItem('currentChapter');
    if (savedChapter) {
      setCurrentChapter(parseInt(savedChapter));
    }
  }, []);

  useEffect(() => {
    // Save progress to localStorage
    localStorage.setItem('bookProgress', JSON.stringify(progress));
  }, [progress]);

  useEffect(() => {
    // Save current chapter to localStorage
    localStorage.setItem('currentChapter', currentChapter.toString());
  }, [currentChapter]);

  const toggleChapter = (chapterId) => {
    setProgress(prev => ({
      ...prev,
      [chapterId]: !prev[chapterId]
    }));
  };

  const completedCount = Object.values(progress).filter(Boolean).length;
  const totalCount = chapters.length;
  const percentage = totalCount > 0 ? Math.round((completedCount / totalCount) * 100) : 0;

  return (
    <div className={styles.progressTracker}>
      <div className={styles.progressBarContainer}>
        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${percentage}%` }}
          ></div>
        </div>
        <div className={styles.progressText}>
          {percentage}% Complete ({completedCount}/{totalCount} chapters)
        </div>
      </div>

      <div className={styles.chapterList}>
        {chapters.map((chapter) => (
          <div
            key={chapter.id}
            className={clsx(
              styles.chapterItem,
              { [styles.completed]: progress[chapter.id] },
              { [styles.current]: currentChapter === chapter.id }
            )}
            onClick={() => toggleChapter(chapter.id)}
            role="button"
            tabIndex={0}
            onKeyDown={(e) => {
              if (e.key === 'Enter' || e.key === ' ') {
                toggleChapter(chapter.id);
              }
            }}
            aria-pressed={progress[chapter.id]}
          >
            <div className={styles.chapterNumber}>
              {progress[chapter.id] ? '✓' : chapter.id}
            </div>
            <div className={styles.chapterTitle}>
              {chapter.title}
            </div>
            {progress[chapter.id] && (
              <span className={styles.completedLabel}>Completed</span>
            )}
          </div>
        ))}
      </div>
    </div>
  );
};

export default ProgressTracker;