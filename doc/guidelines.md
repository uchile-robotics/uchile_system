
# Guidelines y Buenas Prácticas


## Uso de Git

### Nomenclatura de Ramas

- Nombres en `underscore_case`: minúsculas y separados por espacios.
- Usar prefijo `feat-` para nuevas funcionalidades.
- Usar prefijo `rc-` sólo para pruebas específicas de la robocup.

### Merges y otros comandos no recomendados

Evitar a toda costa el uso de `git merge <rama> -s ours` o sus variantes. Con tal comando es posible eliminar el trabajo de otras personas, en caso de haber conflictos. ESO ES MALO!. Los conflictos **deben ser resueltos manualmente**.

De manera similar, se deben tomar en cuenta las siguientes consideraciones:

- Evitar el uso de variantes de "git merge", especialmente las de parámetros "ours" o "theirs".
- **SE PROHIBE** el uso de "git rebase" o "git revert", pues tienen un efecto destructivo en el historial de git.
- Si realmente consideras que debes utilizar una versión de los comandos anteriores, replantea tus motivos. Si aún así consideras que es necesario, consúltalo con quien esté a cargo del sistema.
- Acostúmbrate a hacer commits muy seguido y con mensajes descriptivos.... los commits son baratos y al ser pequeños ayudan a trackear problemas, en vez de convertirse en un montón de cambios que nadie entenderá.
- Acostúmbrate a hacer merge de develop a tu rama en cada sesión de trabajo, así evitas lidiar con infinitos conflictos cuando llegue el momento de llevar tu rama de vuelta a develop.

PS. Se espera que próximamente hayan hooks de git (similares a los que chequean el código al hacer commit), que eviten usos de rebase, revert y variantes del merge.


## Python

Recuerda agregar lo siguiente en la cabecera de los python que crees, para evitar problemas de encoding. Así, se podrán utilizar caracteres especiales sin problemas, como por ejemplo: `á, é, í, ó, ú, ñ`.

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
```
