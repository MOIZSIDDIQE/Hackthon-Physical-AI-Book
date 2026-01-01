import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './ChatBot.css';
import { motion, AnimatePresence } from 'framer-motion';
import { FaCommentDots, FaRobot, FaTimes, FaPaperPlane } from 'react-icons/fa';

const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 1,
      type: 'bot',
      content: 'Hello 👋 I\'m your Physical AI & Humanoid Robotics assistant. Ask me about perception, control, simulation, or any chapter of the book!',
      sources: []
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [suggestedQuestions, setSuggestedQuestions] = useState([
    'What are the fundamentals of Physical AI?',
    'Explain humanoid robot design principles',
    'How does robot perception work?',
    'What are the key control systems for humanoid robots?'
  ]);

  // Handle SSR by checking if we're on the client
  const [isClient, setIsClient] = useState(false);
  const [colorMode, setColorMode] = useState('light'); // Default to light mode

  // Animation variants
  const containerVariants = {
    hidden: { opacity: 0, scale: 0.8 },
    visible: {
      opacity: 1,
      scale: 1,
      transition: {
        type: "spring",
        stiffness: 100,
        damping: 15,
        staggerChildren: 0.1
      }
    },
    exit: {
      opacity: 0,
      scale: 0.8,
      transition: { duration: 0.2 }
    }
  };

  const messageVariants = {
    hidden: {
      opacity: 0,
      y: 20,
      scale: 0.95
    },
    visible: {
      opacity: 1,
      y: 0,
      scale: 1,
      transition: {
        type: "spring",
        stiffness: 100,
        damping: 15
      }
    }
  };

  const inputVariants = {
    focus: {
      scale: 1.02,
      boxShadow: "0 0 0 3px rgba(59, 130, 246, 0.3)"
    },
    blur: {
      scale: 1,
      boxShadow: "none"
    }
  };

  useEffect(() => {
    setIsClient(true);

    // Get the color mode from document class (set by Docusaurus)
    const isDarkMode = document.documentElement.classList.contains('dark');
    setColorMode(isDarkMode ? 'dark' : 'light');

    // Listen for color mode changes
    const observer = new MutationObserver((mutations) => {
      mutations.forEach((mutation) => {
        if (mutation.type === 'attributes' && mutation.attributeName === 'class') {
          const isDark = document.documentElement.classList.contains('dark');
          setColorMode(isDark ? 'dark' : 'light');
        }
      });
    });

    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['class'],
    });

    return () => observer.disconnect();
  }, []);

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      type: 'user',
      content: inputValue.trim(),
      sources: []
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Get API endpoint - use dynamic URL based on environment
      let apiEndpoint;
      if (typeof window !== 'undefined') {
        // Client-side (browser)
        const currentOrigin = window.location.origin;
        // Use the same origin for production, localhost:3001 for local development
        if (currentOrigin.includes('localhost') || currentOrigin.includes('127.0.0.1')) {
          apiEndpoint = 'http://localhost:3001/api/chat'; // Default to original API
        } else {
          apiEndpoint = '/api/chat'; // For deployed environments (Vercel will handle this)
        }
      } else {
        // Server-side (SSR)
        apiEndpoint = process.env.API_URL || 'http://localhost:3001/api/chat';
      }

      // Call the API
      const response = await fetch(apiEndpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue.trim(),
          history: messages.map(m => ({
            role: m.type === 'user' ? 'user' : 'assistant',
            content: m.content
          }))
        }),
      });

      const data = await response.json();

      if (response.ok) {
        const botMessage = {
          id: Date.now() + 1,
          type: 'bot',
          content: data.response,
          sources: data.sources || []
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          type: 'bot',
          content: 'Sorry, I encountered an error. Please try again.',
          sources: []
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        type: 'bot',
        content: 'Sorry, I\'m having trouble connecting. Please check your connection and try again.',
        sources: []
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSuggestedQuestion = (question) => {
    setInputValue(question);
    inputRef.current?.focus();
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  // Only render on client side to avoid SSR issues
  if (!isClient) {
    return null;
  }

  return (
    <>
      {/* Floating Chat Button */}
      {!isOpen && (
        <motion.button
          className={`chatbot-float-button chatbot-float-button--${colorMode}`}
          onClick={toggleChat}
          aria-label="Open chat with Physical AI assistant"
          whileHover={{ scale: 1.1, rotate: 5 }}
          whileTap={{ scale: 0.9 }}
          initial={{ scale: 0, opacity: 0 }}
          animate={{ scale: 1, opacity: 1 }}
          exit={{ scale: 0, opacity: 0 }}
          transition={{ type: "spring", stiffness: 300, damping: 20 }}
        >
          <FaCommentDots size={24} />
          <span className="chatbot-float-button__badge">AI</span>
        </motion.button>
      )}

      {/* Chat Modal */}
      {isOpen && (
        <motion.div
          className="chatbot-modal"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
          transition={{ duration: 0.2 }}
        >
          <motion.div
            className={`chatbot-container chatbot-container--${colorMode}`}
            variants={containerVariants}
            initial="hidden"
            animate="visible"
            exit="exit"
            whileHover={{ scale: 1.01 }}
            transition={{ type: "spring", stiffness: 100, damping: 20 }}
          >
            {/* Chat Header */}
            <div className="chatbot-header">
              <div className="chatbot-header__info">
                <div className="chatbot-header__icon">
                  <FaRobot size={24} />
                </div>
                <div>
                  <h3 className="chatbot-header__title">Physical AI Assistant</h3>
                  <p className="chatbot-header__subtitle">Academic Q&A for Humanoid Robotics</p>
                </div>
              </div>
              <motion.button
                className="chatbot-header__close"
                onClick={toggleChat}
                aria-label="Close chat"
                whileHover={{ scale: 1.2, rotate: 90 }}
                whileTap={{ scale: 0.8 }}
                transition={{ type: "spring", stiffness: 400, damping: 17 }}
              >
                <FaTimes size={20} />
              </motion.button>
            </div>

            {/* Chat Messages */}
            <div className="chatbot-messages">
              <AnimatePresence>
                {messages.map((message) => (
                  <motion.div
                    key={message.id}
                    className={`chatbot-message chatbot-message--${message.type} chatbot-message--${colorMode}`}
                    variants={messageVariants}
                    initial="hidden"
                    animate="visible"
                    exit="hidden"
                    layout
                  >
                    <div className="chatbot-message__content">
                      {message.content.split('\n').map((line, i) => (
                        <p key={i}>{line}</p>
                      ))}
                    </div>

                    {/* Sources for bot messages */}
                    {message.type === 'bot' && message.sources && message.sources.length > 0 && (
                      <div className="chatbot-sources">
                        <small className="chatbot-sources__label">📚 Source:</small>
                        <ul className="chatbot-sources__list">
                          {message.sources.slice(0, 3).map((source, idx) => (
                            <li key={idx} className="chatbot-sources__item">
                              <span className="source-part">{source.part}</span>
                              {source.chapter && <span className="source-chapter"> • {source.chapter}</span>}
                              {source.heading_title && <span className="source-section"> • {source.heading_title}</span>}
                            </li>
                          ))}
                          {message.sources.length > 3 && (
                            <li className="chatbot-sources__item">
                              ... and {message.sources.length - 3} more
                            </li>
                          )}
                        </ul>
                      </div>
                    )}
                  </motion.div>
                ))}
              </AnimatePresence>

              {isLoading && (
                <motion.div
                  className={`chatbot-message chatbot-message--bot chatbot-message--${colorMode}`}
                  variants={messageVariants}
                  initial="hidden"
                  animate="visible"
                  exit="hidden"
                  layout
                >
                  <div className="chatbot-message__content">
                    <div className="typing-indicator">
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </motion.div>
              )}
              <div ref={messagesEndRef} />
            </div>

            {/* Suggested Questions */}
            {messages.length <= 2 && suggestedQuestions.length > 0 && (
              <motion.div
                className="chatbot-suggested-questions"
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.2, duration: 0.3 }}
              >
                {suggestedQuestions.map((question, index) => (
                  <motion.button
                    key={index}
                    className="chatbot-suggested-question"
                    onClick={() => handleSuggestedQuestion(question)}
                    disabled={isLoading}
                    whileHover={{ scale: 1.05, y: -2 }}
                    whileTap={{ scale: 0.95 }}
                    initial={{ opacity: 0, y: 10 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: 0.1 * index }}
                  >
                    {question}
                  </motion.button>
                ))}
              </motion.div>
            )}

            {/* Chat Input */}
            <motion.form
              onSubmit={handleSubmit}
              className="chatbot-input-form"
              variants={inputVariants}
              animate={inputValue ? "focus" : "blur"}
            >
              <motion.input
                ref={inputRef}
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder="Ask about Physical AI, Humanoid Robotics, Perception, Control..."
                disabled={isLoading}
                className="chatbot-input"
                aria-label="Type your question about Physical AI & Humanoid Robotics"
                whileFocus={{ scale: 1.02 }}
                transition={{ type: "spring", stiffness: 300, damping: 30 }}
              />
              <motion.button
                type="submit"
                disabled={!inputValue.trim() || isLoading}
                className="chatbot-send-button"
                aria-label="Send message"
                whileHover={{ scale: 1.1 }}
                whileTap={{ scale: 0.9 }}
                transition={{ type: "spring", stiffness: 400, damping: 17 }}
              >
                <FaPaperPlane size={20} />
              </motion.button>
            </motion.form>
          </motion.div>
        </motion.div>
      )}
    </>
  );
};

export default ChatBot;