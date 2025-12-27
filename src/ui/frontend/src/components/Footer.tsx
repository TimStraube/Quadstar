import React from 'react';

const Footer: React.FC = () => {
  return (
    <>
      <div className="footer-bar">
        <div className="footer-links-left">
          <a href="/simulation" className="footer-link">Simulation</a>
          <a href="/" className="footer-link">Landing</a>
        </div>
        <div className="footer-links-right">
          <a href="/imprint" className="footer-link">Impressum</a>
          <a href="/contact" className="footer-link">Kontakt</a>
        </div>
      </div>

      <div className="footer-copy">
        <small>{new Date().getFullYear()} Thetafly</small>
      </div>
    </>
  );
};

export default Footer;
