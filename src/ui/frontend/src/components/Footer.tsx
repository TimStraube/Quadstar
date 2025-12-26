import React from 'react';

const Footer: React.FC = () => {
  return (
    <>
      <div style={{position: 'fixed', left: 0, right: 0, bottom: 0, zIndex: 50, display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '12px 20px', background: 'linear-gradient(180deg, rgba(0,0,0,0.0), rgba(0,0,0,0.6))'}}>
        <div style={{display: 'flex', gap: 18, alignItems: 'center'}}>
          {/* <div style={{color: 'rgba(255,255,255,0.9)', fontWeight: 600, marginRight: 8}}>Product</div> */}
          <a href="/simulation" style={{color: 'rgba(255,255,255,0.9)', textDecoration: 'none'}}>Simulation</a>
          <a href="/" style={{color: 'rgba(255,255,255,0.9)', textDecoration: 'none'}}>Landing</a>
        </div>
        <div style={{display: 'flex', gap: 18, alignItems: 'center'}}>
          <a href="/imprint" style={{color: 'rgba(255,255,255,0.8)', textDecoration: 'none'}}>Impressum</a>
          <a href="/contact" style={{color: 'rgba(255,255,255,0.8)', textDecoration: 'none'}}>Kontakt</a>
        </div>
      </div>

      <div style={{position: 'fixed', left: 0, right: 0, bottom: 12, display: 'flex', justifyContent: 'center', width: '100%', zIndex: 60, pointerEvents: 'none'}}>
        <small style={{color: 'rgba(255,255,255,0.6)'}}>© {new Date().getFullYear()} Thetafly</small>
      </div>
    </>
  );
};

export default Footer;
