import React from 'react';
import Header from './Header';
import Footer from './Footer';
import { useSelector } from 'react-redux';

const Layout = ({ children }) => {
  return (
    <div className="min-h-screen flex flex-col">
      <Header />
      <main className="flex-grow bg-gray-100 dark:bg-gray-900 py-6 px-4">
        <div className="container mx-auto">{children}</div>
      </main>
      <Footer />
    </div>
  );
};

export default Layout;