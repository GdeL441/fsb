// 'use strict';
var flg = '0';
document.addEventListener('DOMContentLoaded', function() {
  // remove pre-loader start
  setTimeout(function() {
    document.querySelector('.loader-bg').remove();
  }, 400);

  initScrollSpy();
});

function initScrollSpy() {
  const sections = document.querySelectorAll('.pc-content[id]');
  const navItems = document.querySelectorAll('.pc-navbar .pc-item a');
  
  let scrollTimeout;
  
  function updateActiveSection() {
    const scrollPosition = window.scrollY + 100;

    if (scrollPosition <= 150) {
      updateActiveMenuItem('dashboard');
      return;
    }

    let currentSection = null;
    sections.forEach(section => {
      const sectionTop = section.offsetTop;
      const sectionBottom = sectionTop + section.offsetHeight;
      
      if (scrollPosition >= sectionTop && scrollPosition < sectionBottom) {
        currentSection = section.id;
      }
    });

    if (currentSection) {
      updateActiveMenuItem(currentSection);
    }
  }

  function updateActiveMenuItem(sectionId) {
    navItems.forEach(item => {
      item.parentNode.classList.remove('active');
      if (item.parentNode.parentNode.parentNode.classList.contains('pc-trigger')) {
        item.parentNode.parentNode.parentNode.classList.remove('pc-trigger');
      }
    });

    const currentItem = document.querySelector(`.pc-navbar a[href="#${sectionId}"]`);
    if (currentItem) {
      currentItem.parentNode.classList.add('active');
      
      const parentSubmenu = currentItem.closest('.pc-submenu');
      if (parentSubmenu) {
        parentSubmenu.parentNode.classList.add('pc-trigger');
        parentSubmenu.style.display = 'block';
      }
    }
  }

  window.addEventListener('scroll', function() {
    if (scrollTimeout) {
      window.cancelAnimationFrame(scrollTimeout);
    }
    scrollTimeout = window.requestAnimationFrame(updateActiveSection);
  });

  updateActiveSection();
}
