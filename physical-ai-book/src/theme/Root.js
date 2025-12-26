import React from 'react';
import Root from '@theme-original/Root';
import BackToTop from '@site/src/components/BackToTop';
import ChatBot from '@site/src/components/ChatBot';
import './root.css';

export default function RootWrapper(props) {
  return (
    <>
      <a className="skip-to-content" href="#main-content">Skip to content</a>
      <Root {...props} />
      <BackToTop />
      <ChatBot />
    </>
  );
}