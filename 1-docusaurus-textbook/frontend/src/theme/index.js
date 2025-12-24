// src/theme/index.js
import React from 'react';
import {AuthProvider} from '../contexts/AuthContext';

// Get the original Root component
import OriginalRoot from '@theme-original/Root';

// Wrap the original Root with AuthProvider
export const Root = (props) => {
  return (
    <AuthProvider>
      <OriginalRoot {...props} />
    </AuthProvider>
  );
};