(() => {
    const defaultConfig = Object.freeze({
        size: 8,
        squareSize: 60,
        darkColor: '#000000',
        lightColor: '#ffffff',
        boardMargin: 20,
        showCoordinates: false
    });

    const state = { ...defaultConfig };

    const elements = {
        size: document.getElementById('board-size'),
        squareSize: document.getElementById('square-size'),
        darkColor: document.getElementById('dark-color'),
        lightColor: document.getElementById('light-color'),
        boardMargin: document.getElementById('board-margin'),
        showCoordinates: document.getElementById('show-coordinates'),
        resetButton: document.getElementById('reset-config'),
        form: document.getElementById('config-form'),
        board: document.getElementById('chessboard'),
        boardContainer: document.getElementById('board-container'),
        boardWrapper: document.querySelector('.board-wrapper'),
        ipLabel: document.getElementById('local-ip'),
        toggleButton: document.getElementById('toggle-config'),
        controlsPanel: document.getElementById('controls-panel')
    };

    const localIpFromDataset = document.body.dataset.localIp;
    const mobileMediaQuery = window.matchMedia('(max-width: 960px)');

    async function fetchServerInfo() {
        if (localIpFromDataset && localIpFromDataset !== '{{LOCAL_IP}}') {
            elements.ipLabel.textContent = localIpFromDataset;
            return;
        }

        try {
            const response = await fetch('/api/info');
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}`);
            }
            const data = await response.json();
            elements.ipLabel.textContent = data.local_ip ?? 'Không xác định';
        } catch (error) {
            console.error('Không thể tải thông tin IP:', error);
            elements.ipLabel.textContent = 'Không xác định';
        }
    }

    function clamp(value, min, max) {
        return Math.min(Math.max(value, min), max);
    }

    function parseConfig() {
        state.size = clamp(Number(elements.size.value) || defaultConfig.size, 2, 30);
        state.squareSize = clamp(Number(elements.squareSize.value) || defaultConfig.squareSize, 10, 200);
        state.boardMargin = clamp(Number(elements.boardMargin.value) || defaultConfig.boardMargin, 0, 200);
        state.darkColor = elements.darkColor.value || defaultConfig.darkColor;
        state.lightColor = elements.lightColor.value || defaultConfig.lightColor;
        state.showCoordinates = Boolean(elements.showCoordinates.checked);
    }

    function updateBoardSizing() {
        const { boardWrapper, boardContainer, board, boardMargin: marginInput } = elements;
        if (!boardWrapper || !boardContainer || !board) {
            return;
        }

        const bounds = boardWrapper.getBoundingClientRect();
        if (!Number.isFinite(bounds.width) || !Number.isFinite(bounds.height)) {
            return;
        }

        const styles = window.getComputedStyle(boardWrapper);
        const paddingX =
            (Number.parseFloat(styles.paddingLeft) || 0) + (Number.parseFloat(styles.paddingRight) || 0);
        const paddingY =
            (Number.parseFloat(styles.paddingTop) || 0) + (Number.parseFloat(styles.paddingBottom) || 0);

        const innerWidth = bounds.width - paddingX;
        const innerHeight = bounds.height - paddingY;

        if (innerWidth <= 0 || innerHeight <= 0) {
            return;
        }

        const minDimension = Math.min(innerWidth, innerHeight);

        const maxMargin = Math.max(Math.floor(minDimension / 2) - 1, 0);
        const effectiveMargin = clamp(state.boardMargin, 0, maxMargin);

        if (effectiveMargin !== state.boardMargin) {
            state.boardMargin = effectiveMargin;
            if (marginInput) {
                marginInput.value = effectiveMargin;
            }
        }

        const squareSpace = Math.max(minDimension - effectiveMargin * 2, 0);
        if (squareSpace <= 0) {
            return;
        }

        const desiredWidth = state.size * state.squareSize;
        const desiredHeight = state.size * state.squareSize;
        const ratioLimit = Math.min(squareSpace / desiredWidth, squareSpace / desiredHeight);
        const scale = Number.isFinite(ratioLimit) && ratioLimit > 0 ? ratioLimit : 1;

        const boardWidth = desiredWidth * scale;
        const boardHeight = desiredHeight * scale;
        if (!Number.isFinite(boardWidth) || !Number.isFinite(boardHeight) || boardWidth <= 0 || boardHeight <= 0) {
            return;
        }

        const cellSize = Math.min(boardWidth / state.size, boardHeight / state.size);
        const coordFontSize = Math.max(cellSize * 0.32, 10);

        board.style.gridTemplateColumns = `repeat(${state.size}, 1fr)`;
        board.style.gridTemplateRows = `repeat(${state.size}, 1fr)`;
        board.style.width = `${boardWidth}px`;
        board.style.height = `${boardHeight}px`;

        if (state.showCoordinates) {
            board.style.setProperty('--coord-font-size', `${coordFontSize}px`);
        } else {
            board.style.removeProperty('--coord-font-size');
        }

        const squareDimension = squareSpace + effectiveMargin * 2;
        boardContainer.style.width = `${squareDimension}px`;
        boardContainer.style.height = `${squareDimension}px`;
        boardContainer.style.padding = `${effectiveMargin}px`;
    }

    function generateBoard() {
        const { size, darkColor, lightColor, showCoordinates } = state;
        elements.board.innerHTML = '';
        elements.board.style.gridTemplateColumns = `repeat(${size}, 1fr)`;
        elements.board.style.gridTemplateRows = `repeat(${size}, 1fr)`;

        const fragment = document.createDocumentFragment();

        for (let r = 0; r < size; r += 1) {
            for (let c = 0; c < size; c += 1) {
                const square = document.createElement('div');
                const isDark = (r + c) % 2 === 0;
                square.className = `chess-square ${isDark ? 'dark' : 'light'}`;
                square.style.backgroundColor = isDark ? darkColor : lightColor;

                if (showCoordinates) {
                    square.textContent = `${r + 1},${c + 1}`;
                }

                fragment.appendChild(square);
            }
        }

        elements.board.appendChild(fragment);
        if (!showCoordinates) {
            elements.board.style.removeProperty('--coord-font-size');
        }
        updateBoardSizing();
    }

    function syncInputs() {
        elements.size.value = state.size;
        elements.squareSize.value = state.squareSize;
        elements.darkColor.value = state.darkColor;
        elements.lightColor.value = state.lightColor;
        elements.boardMargin.value = state.boardMargin;
        elements.showCoordinates.checked = state.showCoordinates;
    }

    function resetConfig() {
        Object.assign(state, defaultConfig);
        syncInputs();
        generateBoard();
    }

    function setControlsOpen(open, { focusPanel = false } = {}) {
        if (!elements.toggleButton || !elements.controlsPanel) {
            return;
        }
        if (!mobileMediaQuery.matches) {
            document.body.classList.remove('controls-open');
            elements.controlsPanel.removeAttribute('aria-hidden');
            elements.toggleButton.setAttribute('aria-expanded', 'false');
            return;
        }

        document.body.classList.toggle('controls-open', open);
        elements.controlsPanel.setAttribute('aria-hidden', (!open).toString());
        elements.toggleButton.setAttribute('aria-expanded', open.toString());

        if (open && focusPanel) {
            elements.controlsPanel.focus();
        }

        window.requestAnimationFrame(() => {
            updateBoardSizing();
        });
    }

    function closeControls() {
        if (!elements.toggleButton || !mobileMediaQuery.matches) {
            return;
        }
        if (document.body.classList.contains('controls-open')) {
            setControlsOpen(false);
        }
    }

    elements.form.addEventListener('input', () => {
        parseConfig();
        generateBoard();
    });

    elements.resetButton.addEventListener('click', () => {
        resetConfig();
    });

    window.addEventListener('keydown', (event) => {
        if (event.code === 'KeyF' && (event.ctrlKey || event.metaKey)) {
            event.preventDefault();
            const docEl = document.documentElement;
            if (!document.fullscreenElement) {
                docEl.requestFullscreen().catch((err) => {
                    console.warn('Không bật được fullscreen:', err);
                });
            } else {
                document.exitFullscreen().catch(() => {});
            }
        }
    });

    elements.toggleButton?.addEventListener('click', () => {
        const open = !document.body.classList.contains('controls-open');
        setControlsOpen(open, { focusPanel: open });
    });

    document.addEventListener('keydown', (event) => {
        if (event.key === 'Escape') {
            closeControls();
        }
    });

    document.addEventListener('pointerdown', (event) => {
        if (!mobileMediaQuery.matches || !document.body.classList.contains('controls-open')) {
            return;
        }
        if (elements.controlsPanel?.contains(event.target) || elements.toggleButton?.contains(event.target)) {
            return;
        }
        setControlsOpen(false);
    });

    mobileMediaQuery.addEventListener('change', () => {
        if (!mobileMediaQuery.matches) {
            document.body.classList.remove('controls-open');
            elements.controlsPanel?.removeAttribute('aria-hidden');
            elements.toggleButton?.setAttribute('aria-expanded', 'false');
            updateBoardSizing();
        } else {
            setControlsOpen(false);
        }
    });

    window.addEventListener('resize', () => {
        updateBoardSizing();
    });

    window.addEventListener('orientationchange', () => {
        updateBoardSizing();
    });

    fetchServerInfo();
    parseConfig();
    syncInputs();
    generateBoard();
    setControlsOpen(false);
})();
