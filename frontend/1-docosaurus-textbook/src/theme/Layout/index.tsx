import React from 'react';
import Layout from '@theme-original/Layout';
import { ChatProvider } from '@site/src/components/chat/ChatProvider';
import ChatWidget from '@site/src/components/chat';

/**
 * Custom Layout Wrapper
 *
 * This wraps the default Docusaurus Layout with our ChatProvider
 * to enable global chat functionality across all pages.
 *
 * The ChatWidget is rendered as a sibling to the main content,
 * making it available on every page of the site.
 */
export default function LayoutWrapper(props: any) {
  return (
    <ChatProvider>
      <Layout {...props}>
        {props.children}
      </Layout>
      <ChatWidget />
    </ChatProvider>
  );
}
