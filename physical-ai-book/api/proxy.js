// api/proxy.js - Simple proxy to handle API requests
const express = require('express');
const { createProxyMiddleware } = require('http-proxy-middleware');

const router = express.Router();

// Proxy for chat API
router.use('/api/chat', createProxyMiddleware({
  target: process.env.API_SERVER_URL || 'http://localhost:3001',
  changeOrigin: true,
  pathRewrite: {
    '^/api/chat': '/api/chat', // Remove /api/chat prefix when forwarding
  },
  onProxyReq: (proxyReq, req, res) => {
    console.log(`Proxying request: ${req.method} ${req.url}`);
  },
  onProxyRes: (proxyRes, req, res) => {
    console.log(`Proxy response: ${proxyRes.statusCode} for ${req.url}`);
  }
}));

module.exports = router;