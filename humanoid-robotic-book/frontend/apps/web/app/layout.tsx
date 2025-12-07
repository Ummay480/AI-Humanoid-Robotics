import type { Metadata } from 'next';
import './globals.css';

export const metadata: Metadata = {
  title: 'Physical AI & Humanoid Robotics',
  description: 'An AI-Native Interactive Textbook',
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <head>
        <meta charSet="utf-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1" />
        <meta name="theme-color" content="#000000" />
      </head>
      <body className="bg-white dark:bg-slate-950 text-slate-900 dark:text-slate-50 transition-colors">
        {children}
      </body>
    </html>
  );
}
