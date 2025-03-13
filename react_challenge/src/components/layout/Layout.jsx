import React from 'react';
import Header from './Header';
import Footer from './Footer';
import { useSelector } from 'react-redux';

const Layout = ({ children }) => {
  const darkMode = useSelector(state => state.theme.darkMode);
  
  return (
    <div className={`min-h-screen flex flex-col ${darkMode ? 'dark bg-gray-900' : 'bg-gray-100'}`}>
      <Header />
      <main className="flex-grow py-6 px-4">
        <div className="container mx-auto">
          {children}
        </div>
      </main>
      <Footer />
    </div>
  );
};

export default Layout;