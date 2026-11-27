from typing import Dict, List


def build_sparql_query(category: str = '', exclude_flavor: str = '') -> str:
    """Assemble deterministic SPARQL query from structured slots."""
    filters: List[str] = []
    if category:
        filters.append(f'FILTER(LCASE(?category) = "{category.lower()}")')
    if exclude_flavor:
        filters.append(f'FILTER(LCASE(?flavor) != "{exclude_flavor.lower()}")')
    filter_block = '\n  '.join(filters)
    return (
        'PREFIX : <http://store.cobot/ontology#>\n'
        'SELECT ?product ?category ?flavor WHERE {\n'
        '  ?product a :Product ;\n'
        '           :hasCategory ?category ;\n'
        '           :hasFlavor ?flavor .\n'
        f'  {filter_block}\n'
        '}'
    )


def to_networkx_snapshot(items: List[Dict[str, str]]) -> Dict[str, list]:
    """Convert inventory items into lightweight node/edge JSON for Web UI."""
    nodes = [{'id': item['id'], 'label': item.get('name', item['id'])} for item in items]
    return {'nodes': nodes, 'edges': []}
