import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from './contexts/AuthContext';

export default function App() {
  return (
    <AuthProvider>
      <Layout>
        {/* Content will be rendered by Docusaurus */}
      </Layout>
    </AuthProvider>
  );
}