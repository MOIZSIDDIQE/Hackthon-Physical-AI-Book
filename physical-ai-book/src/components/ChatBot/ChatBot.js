import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './ChatBot.css';
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
      // Get API endpoint - explicitly set to the known working URL
      const apiEndpoint = 'http://localhost:3001/api/chat';

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
        <button
          className={`chatbot-float-button chatbot-float-button--${colorMode}`}
          onClick={toggleChat}
          aria-label="Open chat with Physical AI assistant"
        >
          <FaCommentDots size={24} />
          <span className="chatbot-float-button__badge">AI</span>
        </button>
      )}

      {/* Chat Modal */}
      {isOpen && (
        <div className="chatbot-modal">
          <div className={`chatbot-container chatbot-container--${colorMode}`}>
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
              <button
                className="chatbot-header__close"
                onClick={toggleChat}
                aria-label="Close chat"
              >
                <FaTimes size={20} />
              </button>
            </div>

            {/* Chat Messages */}
            <div className="chatbot-messages">
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`chatbot-message chatbot-message--${message.type} chatbot-message--${colorMode}`}
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
                </div>
              ))}

              {isLoading && (
                <div className={`chatbot-message chatbot-message--bot chatbot-message--${colorMode}`}>
                  <div className="chatbot-message__content">
                    <div className="typing-indicator">
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>

            {/* Suggested Questions */}
            {messages.length <= 2 && suggestedQuestions.length > 0 && (
              <div className="chatbot-suggested-questions">
                {suggestedQuestions.map((question, index) => (
                  <button
                    key={index}
                    className="chatbot-suggested-question"
                    onClick={() => handleSuggestedQuestion(question)}
                    disabled={isLoading}
                  >
                    {question}
                  </button>
                ))}
              </div>
            )}

            {/* Chat Input */}
            <form onSubmit={handleSubmit} className="chatbot-input-form">
              <input
                ref={inputRef}
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder="Ask about Physical AI, Humanoid Robotics, Perception, Control..."
                disabled={isLoading}
                className="chatbot-input"
                aria-label="Type your question about Physical AI & Humanoid Robotics"
              />
              <button
                type="submit"
                disabled={!inputValue.trim() || isLoading}
                className="chatbot-send-button"
                aria-label="Send message"
              >
                <FaPaperPlane size={20} />
              </button>
            </form>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatBot;